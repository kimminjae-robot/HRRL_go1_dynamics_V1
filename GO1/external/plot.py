import socket
import threading
import tkinter as tk
from tkinter import ttk
from collections import deque
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.animation as animation
from matplotlib.figure import Figure
from itertools import cycle


class TCPClient:
    def __init__(self, host='127.0.0.1', port=9000):
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.buffer = ''
        self.lock = threading.Lock()
        self.lines = deque()

    def start(self):
        try:
            self.sock.connect(self.addr)
            threading.Thread(target=self._recv_loop, daemon=True).start()
            print(f"Connected to TCP server at {self.addr}")
        except ConnectionRefusedError:
            print(f"Connection refused: Could not connect to {self.addr}. Is the server running?")
            exit()

    def _recv_loop(self):
        while True:
            try:
                data = self.sock.recv(1024).decode('utf-8')
                if not data:
                    print("Server disconnected.")
                    break
                with self.lock:
                    self.buffer += data
                    while '\n' in self.buffer:
                        line, self.buffer = self.buffer.split('\n', 1)
                        self.lines.append(line)
            except Exception as e:
                print(f"TCP receive error: {e}")
                break

    def get_lines(self):
        with self.lock:
            out = list(self.lines)
            self.lines.clear()
        return out

class LivePlotApp:
    def __init__(self, master):
        self.master = master
        self.master.title("Dynamic TCP Plotter")

        # --- TCP Client ---
        self.client = TCPClient()
        self.client.start()

        # --- Dynamic storage for metrics ---
        self.data           = {}      # key -> deque of floats
        self.lines          = {}      # key -> Line2D
        self.metric_vars    = {}      # key -> BooleanVar
        self.sample_counter = 0
        self.max_len        = 100

        # --- GUI Layout ---
        self.control_frame = ttk.Frame(master)
        self.control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=5, pady=5)
        ttk.Label(self.control_frame, text="Select metrics:").pack(anchor=tk.W)

        # Time horizon
        ttk.Label(self.control_frame, text="Time horizon (samples):").pack(anchor=tk.W, pady=(10,0))
        self.horizon_sb = ttk.Spinbox(
            self.control_frame,
            from_=10, to=5000, increment=10, width=6
        )
        self.horizon_sb.set(self.max_len)
        self.horizon_sb.pack(anchor=tk.W)

        # Y-axis min/max
        ttk.Label(self.control_frame, text="Y-axis min:").pack(anchor=tk.W, pady=(10,0))
        self.ymin_sb = ttk.Spinbox(
            self.control_frame,
            from_=-100.0, to=100.0, increment=0.1, width=6
        )
        self.ymin_sb.set(-1.0)
        self.ymin_sb.pack(anchor=tk.W)

        ttk.Label(self.control_frame, text="Y-axis max:").pack(anchor=tk.W, pady=(5,0))
        self.ymax_sb = ttk.Spinbox(
            self.control_frame,
            from_=-100.0, to=100.0, increment=0.1, width=6
        )
        self.ymax_sb.set(1.0)
        self.ymax_sb.pack(anchor=tk.W)

        ttk.Button(self.control_frame, text="Apply", command=self.apply_settings).pack(pady=(10,0))

        # --- Matplotlib Figure ---
        self.fig = Figure(figsize=(6,4))
        self.ax  = self.fig.add_subplot(111)
        self.ax.set_xlabel('Sample #')
        self.ax.set_ylabel('Value')
        self.ax.grid(True)

        self.canvas = FigureCanvasTkAgg(self.fig, master=master)
        self.canvas.get_tk_widget().pack(side=tk.RIGHT, fill=tk.BOTH, expand=1)

        # color cycler for new metrics
        colors = plt.cm.get_cmap('tab20').colors
        self.color_cycler = cycle(colors)
        # self.color_cycler = iter(plt.rcParams['axes.prop_cycle'].by_key()['color'])

        # animation
        self.ani = animation.FuncAnimation(
            self.fig, self.update_plot, init_func=self.init_plot,
            interval=50, blit=False
        )

    def apply_settings(self):
        # 1) Time horizon
        try:
            new_h = int(self.horizon_sb.get())
            if new_h != self.max_len:
                self.max_len = new_h
                for k in self.data:
                    old = list(self.data[k])
                    self.data[k] = deque(old, maxlen=self.max_len)
                print(f"Horizon -> {self.max_len}")
        except ValueError:
            pass

        # 2) Y-axis limits
        try:
            ymin = float(self.ymin_sb.get())
            ymax = float(self.ymax_sb.get())
            if ymin >= ymax:
                print("Error: Y-min must be < Y-max")
            else:
                self.ax.set_ylim(ymin, ymax)
                print(f"Y-axis -> [{ymin}, {ymax}]")
                self.canvas.draw_idle()
        except ValueError:
            pass

    def init_plot(self):
        # initial axes limits
        self.ax.set_xlim(0, self.max_len)
        ymin = float(self.ymin_sb.get())
        ymax = float(self.ymax_sb.get())
        self.ax.set_ylim(ymin, ymax)
        return list(self.lines.values())

    def add_metric(self, key):
        """새로운 키가 감지되면 GUI와 plot에 추가."""
        # 1) data deque
        self.data[key] = deque(maxlen=self.max_len)

        # 2) checkbox
        var = tk.BooleanVar(value=False)
        chk = ttk.Checkbutton(self.control_frame, text=key, variable=var)
        chk.pack(anchor=tk.W)
        self.metric_vars[key] = var

        # 3) plot line
        color = next(self.color_cycler)
        line, = self.ax.plot([], [], label=key, color=color)
        self.lines[key] = line

        # 4) legend 갱신
        self.ax.legend(loc='upper right')
        print(f"Added metric: {key}")

    def update_plot(self, frame):
        raws   = self.client.get_lines()
        updated = False

        for raw in raws:
            try:
                parts = dict(item.split('=') for item in raw.split(','))
            except Exception:
                continue

            # 새로운 키 자동 추가
            for k in parts:
                if k not in self.data:
                    self.add_metric(k)

            # 값 append
            for k, v in parts.items():
                try:
                    self.data[k].append(float(v))
                except ValueError:
                    pass

            self.sample_counter += 1
            updated = True

        if updated:
            x_end   = self.sample_counter
            x_start = max(0, x_end - self.max_len)

            for k, line in self.lines.items():
                if self.metric_vars[k].get():
                    y = list(self.data[k])
                    x = list(range(x_end - len(y), x_end))
                    line.set_data(x, y)
                else:
                    line.set_data([], [])

            self.ax.set_xlim(x_start, x_end)
            # Y-axis limits are controlled in apply_settings/init_plot

            self.canvas.draw_idle()

        return list(self.lines.values())

if __name__ == '__main__':
    root = tk.Tk()
    app = LivePlotApp(root)
    root.mainloop()
