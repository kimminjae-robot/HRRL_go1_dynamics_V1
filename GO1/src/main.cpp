
#include <Controllers.hpp>


#define MUJOCO_PLUGIN_DIR "mujoco_plugin"

extern "C" {
#if defined(_WIN32) || defined(__CYGWIN__)
  #include <windows.h>
#else
  #if defined(__APPLE__)
    #include <mach-o/dyld.h>
  #endif
  #include <sys/errno.h>
  #include <unistd.h>
#endif
}

namespace {
namespace mj = ::mujoco;
namespace mju = ::mujoco::sample_util;

// constants
const double syncMisalign = 0.1;        // maximum mis-alignment before re-sync (simulation seconds)
const double simRefreshFraction = 0.7;  // fraction of refresh available for simulation
const int kErrorLength = 1024;          // load error string length

// model and data
mjModel* m = nullptr;
mjData* d = nullptr;

Controllers* controllers = nullptr;
std::atomic<bool> mujoco_ready{false};

using Seconds = std::chrono::duration<double>;

//---------------------------------------- plugin handling -----------------------------------------

// return the path to the directory containing the current executable
// used to determine the location of auto-loaded plugin libraries
std::string getExecutableDir() {
#if defined(_WIN32) || defined(__CYGWIN__)
  constexpr char kPathSep = '\\';
  std::string realpath = [&]() -> std::string {
    std::unique_ptr<char[]> realpath(nullptr);
    DWORD buf_size = 128;
    bool success = false;
    while (!success) {
      realpath.reset(new(std::nothrow) char[buf_size]);
      if (!realpath) {
        std::cerr << "cannot allocate memory to store executable path\n";
        return "";
      }

      DWORD written = GetModuleFileNameA(nullptr, realpath.get(), buf_size);
      if (written < buf_size) {
        success = true;
      } else if (written == buf_size) {
        // realpath is too small, grow and retry
        buf_size *=2;
      } else {
        std::cerr << "failed to retrieve executable path: " << GetLastError() << "\n";
        return "";
      }
    }
    return realpath.get();
  }();
#else
  constexpr char kPathSep = '/';
#if defined(__APPLE__)
  std::unique_ptr<char[]> buf(nullptr);
  {
    std::uint32_t buf_size = 0;
    _NSGetExecutablePath(nullptr, &buf_size);
    buf.reset(new char[buf_size]);
    if (!buf) {
      std::cerr << "cannot allocate memory to store executable path\n";
      return "";
    }
    if (_NSGetExecutablePath(buf.get(), &buf_size)) {
      std::cerr << "unexpected error from _NSGetExecutablePath\n";
    }
  }
  const char* path = buf.get();
#else
  const char* path = "/proc/self/exe";
#endif
  std::string realpath = [&]() -> std::string {
    std::unique_ptr<char[]> realpath(nullptr);
    std::uint32_t buf_size = 128;
    bool success = false;
    while (!success) {
      realpath.reset(new(std::nothrow) char[buf_size]);
      if (!realpath) {
        std::cerr << "cannot allocate memory to store executable path\n";
        return "";
      }

      std::size_t written = readlink(path, realpath.get(), buf_size);
      if (written < buf_size) {
        realpath.get()[written] = '\0';
        success = true;
      } else if (written == -1) {
        if (errno == EINVAL) {
          // path is already not a symlink, just use it
          return path;
        }

        std::cerr << "error while resolving executable path: " << strerror(errno) << '\n';
        return "";
      } else {
        // realpath is too small, grow and retry
        buf_size *= 2;
      }
    }
    return realpath.get();
  }();
#endif

  if (realpath.empty()) {
    return "";
  }

  for (std::size_t i = realpath.size() - 1; i > 0; --i) {
    if (realpath.c_str()[i] == kPathSep) {
      return realpath.substr(0, i);
    }
  }

  // don't scan through the entire file system's root
  return "";
}

// scan for libraries in the plugin directory to load additional plugins
void scanPluginLibraries() {
  // check and print plugins that are linked directly into the executable
  int nplugin = mjp_pluginCount();
  if (nplugin) {
    std::printf("Built-in plugins:\n");
    for (int i = 0; i < nplugin; ++i) {
      std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
    }
  }

  // define platform-specific strings
#if defined(_WIN32) || defined(__CYGWIN__)
  const std::string sep = "\\";
#else
  const std::string sep = "/";
#endif


  // try to open the ${EXECDIR}/MUJOCO_PLUGIN_DIR directory
  // ${EXECDIR} is the directory containing the simulate binary itself
  // MUJOCO_PLUGIN_DIR is the MUJOCO_PLUGIN_DIR preprocessor macro
  const std::string executable_dir = getExecutableDir();
  if (executable_dir.empty()) {
    return;
  }

  const std::string plugin_dir = getExecutableDir() + sep + MUJOCO_PLUGIN_DIR;
  mj_loadAllPluginLibraries(
      plugin_dir.c_str(), +[](const char* filename, int first, int count) {
        std::printf("Plugins registered by library '%s':\n", filename);
        for (int i = first; i < first + count; ++i) {
          std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
        }
      });
}

//------------------------------------------- simulation -------------------------------------------

const char* Diverged(int disableflags, const mjData* d) {
  if (disableflags & mjDSBL_AUTORESET) {
    for (mjtWarning w : {mjWARN_BADQACC, mjWARN_BADQVEL, mjWARN_BADQPOS}) {
      if (d->warning[w].number > 0) {
        return mju_warningText(w, d->warning[w].lastinfo);
      }
    }
  }
  return nullptr;
}

mjModel* LoadModel(const char* file, mj::Simulate& sim) {
  // this copy is needed so that the mju::strlen call below compiles
  char filename[mj::Simulate::kMaxFilenameLength];
  mju::strcpy_arr(filename, file);

  // make sure filename is not empty
  if (!filename[0]) {
    return nullptr;
  }

  // load and compile
  char loadError[kErrorLength] = "";
  mjModel* mnew = 0;
  auto load_start = mj::Simulate::Clock::now();
  if (mju::strlen_arr(filename)>4 &&
      !std::strncmp(filename + mju::strlen_arr(filename) - 4, ".mjb",
                    mju::sizeof_arr(filename) - mju::strlen_arr(filename)+4)) {
    mnew = mj_loadModel(filename, nullptr);
    if (!mnew) {
      mju::strcpy_arr(loadError, "could not load binary model");
    }
  } else {
    mnew = mj_loadXML(filename, nullptr, loadError, kErrorLength);

    // remove trailing newline character from loadError
    if (loadError[0]) {
      int error_length = mju::strlen_arr(loadError);
      if (loadError[error_length-1] == '\n') {
        loadError[error_length-1] = '\0';
      }
    }
  }
  auto load_interval = mj::Simulate::Clock::now() - load_start;
  double load_seconds = Seconds(load_interval).count();

  if (!mnew) {
    std::printf("%s\n", loadError);
    mju::strcpy_arr(sim.load_error, loadError);
    return nullptr;
  }

  // compiler warning: print and pause
  if (loadError[0]) {
    // mj_forward() below will print the warning message
    std::printf("Model compiled, but simulation warning (paused):\n  %s\n", loadError);
    sim.run = 0;
  }

  // if no error and load took more than 1/4 seconds, report load time
  else if (load_seconds > 0.25) {
    mju::sprintf_arr(loadError, "Model loaded in %.2g seconds", load_seconds);
  }

  mju::strcpy_arr(sim.load_error, loadError);

  return mnew;
}

// simulate in background thread (while rendering in main thread)
void PhysicsLoop(mj::Simulate& sim) {
  // cpu-sim syncronization point
  std::chrono::time_point<mj::Simulate::Clock> syncCPU;
  mjtNum syncSim = 0;

  // run until asked to exit
  while (!sim.exitrequest.load()) {
    if (sim.droploadrequest.load()) {
      sim.LoadMessage(sim.dropfilename);
      mjModel* mnew = LoadModel(sim.dropfilename, sim);
      sim.droploadrequest.store(false);

      mjData* dnew = nullptr;
      if (mnew) dnew = mj_makeData(mnew);
      if (dnew) {
        sim.Load(mnew, dnew, sim.dropfilename);

        // lock the sim mutex
        const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

        mj_deleteData(d);
        mj_deleteModel(m);

        m = mnew;
        d = dnew;
        mj_forward(m, d);

      } else {
        sim.LoadMessageClear();
      }
    }

    if (sim.uiloadrequest.load()) {
      sim.uiloadrequest.fetch_sub(1);
      sim.LoadMessage(sim.filename);
      mjModel* mnew = LoadModel(sim.filename, sim);
      mjData* dnew = nullptr;
      if (mnew) dnew = mj_makeData(mnew);
      if (dnew) {
        sim.Load(mnew, dnew, sim.filename);

        // lock the sim mutex
        const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

        mj_deleteData(d);
        mj_deleteModel(m);

        m = mnew;
        d = dnew;
        mj_forward(m, d);

      } else {
        sim.LoadMessageClear();
      }
    }

    // sleep for 1 ms or yield, to let main thread run
    //  yield results in busy wait - which has better timing but kills battery life
    if (sim.run && sim.busywait) {
      std::this_thread::yield();
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    {
      // lock the sim mutex
      const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

      // run only if model is present
      if (m) {
        // running
        if (sim.run) {
          bool stepped = false;

          // record cpu time at start of iteration
          const auto startCPU = mj::Simulate::Clock::now();

          // elapsed CPU and simulation time since last sync
          const auto elapsedCPU = startCPU - syncCPU;
          double elapsedSim = d->time - syncSim;

          // requested slow-down factor
          double slowdown = 100 / sim.percentRealTime[sim.real_time_index];

          // misalignment condition: distance from target sim time is bigger than syncmisalign
          bool misaligned =
              std::abs(Seconds(elapsedCPU).count()/slowdown - elapsedSim) > syncMisalign;

          // out-of-sync (for any reason): reset sync times, step
          if (elapsedSim < 0 || elapsedCPU.count() < 0 || syncCPU.time_since_epoch().count() == 0 ||
              misaligned || sim.speed_changed) {
            // re-sync
            syncCPU = startCPU;
            syncSim = d->time;
            sim.speed_changed = false;

            // run single step, let next iteration deal with timing
            mj_step(m, d);
            const char* message = Diverged(m->opt.disableflags, d);
            if (message) {
              sim.run = 0;
              mju::strcpy_arr(sim.load_error, message);
            } else {
              stepped = true;
            }
          }

          // in-sync: step until ahead of cpu
          else {
            bool measured = false;
            mjtNum prevSim = d->time;

            double refreshTime = simRefreshFraction/sim.refresh_rate;

            // step while sim lags behind cpu and within refreshTime
            while (Seconds((d->time - syncSim)*slowdown) < mj::Simulate::Clock::now() - syncCPU &&
                   mj::Simulate::Clock::now() - startCPU < Seconds(refreshTime)) {
              // measure slowdown before first step
              if (!measured && elapsedSim) {
                sim.measured_slowdown =
                    std::chrono::duration<double>(elapsedCPU).count() / elapsedSim;
                measured = true;
              }

              // inject noise
              sim.InjectNoise();

              // call mj_step
              mj_step(m, d);
              const char* message = Diverged(m->opt.disableflags, d);
              if (message) {
                sim.run = 0;
                mju::strcpy_arr(sim.load_error, message);
              } else {
                stepped = true;
              }

              // break if reset
              if (d->time < prevSim) {
                break;
              }
            }
          }

          // save current state to history buffer
          if (stepped) {
            sim.AddToHistory();
          }
        }

        // paused
        else {
          // run mj_forward, to update rendering and joint sliders
          mj_forward(m, d);
          sim.speed_changed = true;
        }
      }
    }  // release std::lock_guard<std::mutex>
  }
}
}  // namespace

//-------------------------------------- physics_thread --------------------------------------------

void PhysicsThread(mj::Simulate* sim, const char* filename) {
  // request loadmodel if file given (otherwise drag-and-drop)
  if (filename != nullptr) {
    printf("[Debug] 1. Filename received: %s\n", filename);
    sim->LoadMessage(filename);
    printf("[Debug] 2. Calling LoadModel...\n");
    m = LoadModel(filename, *sim);
    printf("[Debug] 3. LoadModel returned. Model pointer: %p\n", m);
    if (m) {
      // lock the sim mutex
      printf("[Debug] 4. Calling mj_makeData...\n");
      const std::unique_lock<std::recursive_mutex> lock(sim->mtx);

      d = mj_makeData(m);
      printf("[Debug] 5. mj_makeData returned. Data pointer: %p\n", d);
    }
    if (d) {
      printf("[Debug] 6. Calling sim->Load...\n");
      sim->Load(m, d, filename);
      printf("[Debug] 7. sim->Load returned.\n");
      // lock the sim mutex
      const std::unique_lock<std::recursive_mutex> lock(sim->mtx);

      mj_forward(m, d);

    } else {
      sim->LoadMessageClear();
    }
  }
  printf("[Debug] 8. End of loading block.\n");
  mujoco_ready = true;
  std::cout << "mujoco_ready flag set to :" << mujoco_ready << std::endl;
  PhysicsLoop(*sim);

  mj_deleteData(d);
  mj_deleteModel(m);
}

void initialize_contollers()
{
  // 공유 메모리 객체 생성 (12개의 값 저장)
  SharedMemory* shared_memory = new SharedMemory(12);
  std::cout << "SharedMemory Ready" << std::endl;
  
  // 데몬(백그라운드 작업) 객체 생성 및 시작
  Daemon* daemon = new Daemon(m, d, shared_memory);
  daemon->start();
  std::cout << "Daemon Ready" << std::endl;
  
  // PD 컨트롤러 초기화
  std::string pd_config_path = "../config/pd_config.yaml";
  YAML::Node pd_config = YAML::LoadFile(pd_config_path);

  // PD 제어주기, 게인 값 등 파라미터 읽기
  double pd_control_frequency = pd_config["control_frequency"].as<double>();
  double kp = pd_config["kp"].as<double>();
  double kd = pd_config["kd"].as<double>();
  double num_joint = pd_config["num_joint"].as<double>();
  //조인트 위치 PD 컨트롤러 객체 생성
  JointPositionPDController* pd_controller = new JointPositionPDController(pd_control_frequency, num_joint, kp, kd);
  std::cout << "JointPositionPDController Ready" << std::endl;

  // 태스크 컨트롤러 생성 및 초기화, 시작 
  TaskController* task_controller = new TaskController(100, shared_memory);
  task_controller->initialize();
  task_controller->start();
  std::cout << "TaskController Ready" << std::endl;

  // 조이스틱 객체 생성 및 초기화, 시작
  Joystick* joystick = new Joystick("/dev/input/js0", shared_memory);
  joystick->initialize();
  joystick->start();

  // 컨트롤러들을 담는 구조체 생성
  controllers = new Controllers;

  // 각 컨트롤러 포인터 할당
  controllers->pd_controller = pd_controller;
  controllers->shared_memory = shared_memory;
  controllers->daemon = daemon;
  controllers->task_controller = task_controller;
  controllers->joystick_ = joystick;

  std::cout << "Controllers Ready" << std::endl;
}

void PositionControlThread(JointPositionPDController* pd_controller, SharedMemory* shared_memory)
{
  // 제어 주기(frequency) 설정
  double frequency = pd_controller->get_control_frequency();
  // 루프 주기(loop_duration) 계산 (마이크로초 단위)
  std::chrono::microseconds loop_duration(static_cast<long long>((1/frequency) * 1000000));

  // 루프 시작/종료 시간 변수 선언
  auto start_time = std::chrono::steady_clock::now();
  auto end_time = std::chrono::steady_clock::now();

  std::printf("PositionControlThread Start");

  while(true)
  {
    // 루프 시작 시간 기록
    start_time = std::chrono::steady_clock::now();

    // 실제 조인트 위치/속도 설정
    pd_controller->set_joint_actual(shared_memory->q, shared_memory->qd);
    // 목표 조인트 위치 설정
    pd_controller->set_joint_target(shared_memory->q_target);
    
    // 토크 계산
    std::vector<double> computed_torque = pd_controller->compute_torque();
    // 계산된 토크를 공유 메모리에 저장
    for (int i=0;i<12;i++)
    {
      shared_memory->torque_target[i] = computed_torque[i];
    }
    
    // 루프 종료 시간 기록
    end_time = std::chrono::steady_clock::now();
    // 루프 실행 시간 계산
    auto elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    // 다음 루프 시작 시간 계산
    auto next_time = start_time + loop_duration;
    // 다음 루프 시작 시간까지 대기
    std::this_thread::sleep_until(next_time);

  }

}

void RunInitializationProcess() {
  // mujoco_ready 플래그가 true가 될 때까지 대기 (시뮬레이터 초기화 대기)
  while (!mujoco_ready)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  std::cout << "Mujoco initialized" << std::endl;
  // 모델(m)과 데이터(d)가 모두 준비된 경우에만 컨트롤러 초기화
  if (m && d) {
    initialize_contollers();
    // 컨트롤러 구조체가 정상적으로 생성되었는지 확인
    if (controllers != nullptr) {
      std::cout << "Initialization thread: Starting PositionControlThread..." << std::endl;
      // 위치 제어 스레드 시작 (PD 제어 루프)
      std::thread position_controller_thread_handle(&PositionControlThread, controllers->pd_controller, controllers->shared_memory);
      position_controller_thread_handle.detach(); // 백그라운드로 실행
    }
  }

  std::cout << "Initialization thread: All tasks complete. This thread will now exit." << std::endl;
}

// run event loop
int main(int argc, char** argv) {

  // print version, check compatibility
  std::printf("MuJoCo version %s\n", mj_versionString());
  if (mjVERSION_HEADER!=mj_version()) {
    mju_error("Headers and library have different versions");
  }

  // scan for libraries in the plugin directory to load additional plugins
  scanPluginLibraries();

  mjvCamera cam;
  mjv_defaultCamera(&cam);

  mjvOption opt;
  mjv_defaultOption(&opt);

  mjvPerturb pert;
  mjv_defaultPerturb(&pert);

  // simulate object encapsulates the UI
  auto sim = std::make_unique<mj::Simulate>(
      std::make_unique<mj::GlfwAdapter>(),
      &cam, &opt, &pert, /* is_passive = */ false
  );

  const char* filename = nullptr;
  if (argc >  1) {
    filename = argv[1];
  }

  // start physics thread
  std::thread physicsthreadhandle(&PhysicsThread, sim.get(), filename);
  
  // Start custom thread --kale--
  std::thread initialization_thread_handle(&RunInitializationProcess);
  initialization_thread_handle.detach();

  // start simulation UI loop (blocking call)
  sim->RenderLoop();

  physicsthreadhandle.join();

  return 0;
}


