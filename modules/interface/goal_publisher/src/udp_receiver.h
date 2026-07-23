#ifndef UDP_RECEIVER_H
#define UDP_RECEIVER_H

#include <string>
#include <optional>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>

class UdpReceiver {
public:
    UdpReceiver(int port);
    ~UdpReceiver();

    bool initialize();
    bool hasData();
    std::optional<std::string> receiveData();

private:
    int socket_fd_;
    int port_;
    struct sockaddr_in server_addr_;
    static constexpr size_t BUFFER_SIZE = 1024;
};

#endif // UDP_RECEIVER_H
