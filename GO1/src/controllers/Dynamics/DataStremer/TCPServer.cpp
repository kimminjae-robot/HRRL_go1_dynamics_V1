// TCPServer.cpp
#include "TCPServer.h"
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <iostream>
#include <cstring>

TCPServer::TCPServer(int port)
    : port_(port), listen_fd_(-1), client_fd_(-1), running_(false) {}

TCPServer::~TCPServer() {
    stop();
}

bool TCPServer::start() {
    listen_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd_ < 0) {
        perror("socket");
        return false;
    }
    int opt = 1;
    setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    server_addr_.sin_family = AF_INET;
    server_addr_.sin_addr.s_addr = INADDR_ANY;
    server_addr_.sin_port = htons(port_);

    if (bind(listen_fd_, reinterpret_cast<sockaddr*>(&server_addr_), sizeof(server_addr_)) < 0) {
        perror("bind");
        return false;
    }
    if (listen(listen_fd_, 1) < 0) {
        perror("listen");
        return false;
    }

    running_ = true;
    accept_thread_ = std::thread(&TCPServer::acceptLoop, this);
    return true;
}

void TCPServer::acceptLoop() {
    while (running_) {
        sockaddr_in client_addr;
        socklen_t len = sizeof(client_addr);
        int fd = accept(listen_fd_, reinterpret_cast<sockaddr*>(&client_addr), &len);
        if (fd >= 0) {
            client_fd_ = fd;
            std::cout << "Client connected: "
                      << inet_ntoa(client_addr.sin_addr) << ":"
                      << ntohs(client_addr.sin_port) << "\n";
            break;
        }
    }
}

bool TCPServer::sendData(const std::string& data) {
    if (client_fd_ < 0) return false;
    ssize_t sent = send(client_fd_, data.c_str(), data.size(), 0);
    return sent == static_cast<ssize_t>(data.size());
}

void TCPServer::stop() {
    running_ = false;
    if (listen_fd_ >= 0)   close(listen_fd_);
    if (client_fd_ >= 0)   close(client_fd_);
    if (accept_thread_.joinable()) accept_thread_.join();
}
