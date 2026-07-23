#include "udp_receiver.h"
#include <iostream>
#include <cstring>
#include <sys/select.h>
#include <errno.h>

UdpReceiver::UdpReceiver(int port)
    : port_(port)
    , socket_fd_(-1)
{
}

UdpReceiver::~UdpReceiver() {
    if (socket_fd_ >= 0) {
        close(socket_fd_);
    }
}

bool UdpReceiver::initialize() {
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
        std::cerr << "Socket creation failed" << std::endl;
        return false;
    }

    std::memset(&server_addr_, 0, sizeof(server_addr_));
    server_addr_.sin_family = AF_INET;
    server_addr_.sin_addr.s_addr = INADDR_ANY;
    server_addr_.sin_port = htons(port_);

    if (bind(socket_fd_, (struct sockaddr*)&server_addr_, sizeof(server_addr_)) < 0) {
        std::cerr << "Bind failed" << std::endl;
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    int flags = fcntl(socket_fd_, F_GETFL, 0);
    if (flags < 0 || fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
        std::cerr << "Failed to set non-blocking mode" << std::endl;
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    std::cout << "UDP receiver initialized on port " << port_ << std::endl;
    return true;
}

bool UdpReceiver::hasData() {
    if (socket_fd_ < 0) {
        return false;
    }

    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(socket_fd_, &read_fds);

    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    int result = select(socket_fd_ + 1, &read_fds, nullptr, nullptr, &timeout);
    return result > 0;
}

std::optional<std::string> UdpReceiver::receiveData() {
    if (socket_fd_ < 0) {
        return std::nullopt;
    }

    char buffer[BUFFER_SIZE];
    struct sockaddr_in client_addr;
    socklen_t client_addr_len = sizeof(client_addr);

    ssize_t bytes_received = recvfrom(socket_fd_, buffer, BUFFER_SIZE - 1, 0,
                                      (struct sockaddr*)&client_addr, &client_addr_len);

    if (bytes_received < 0) {
        if (errno != EWOULDBLOCK && errno != EAGAIN) {
            std::cerr << "recvfrom failed: " << errno << std::endl;
        }
        return std::nullopt;
    }

    if (bytes_received > 0) {
        buffer[bytes_received] = '\0';
        return std::string(buffer, bytes_received);
    }

    return std::nullopt;
}
