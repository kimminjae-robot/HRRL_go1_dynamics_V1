// TCPServer.h
#ifndef TCPSERVER_H
#define TCPSERVER_H

#include <string>
#include <thread>
#include <atomic>
#include <netinet/in.h>

class TCPServer {
public:
    explicit TCPServer(int port);
    ~TCPServer();

    // 서버 시작: 백그라운드로 accept loop 실행
    bool start();
    // 서버 중지: 소켓 닫고 스레드 조인
    void stop();
    // 클라이언트에 데이터 전송
    bool sendData(const std::string& data);

private:
    void acceptLoop();

    int port_;
    int listen_fd_;
    int client_fd_;
    std::thread accept_thread_;
    std::atomic<bool> running_;
    struct sockaddr_in server_addr_;
};

#endif // TCPSERVER_H
