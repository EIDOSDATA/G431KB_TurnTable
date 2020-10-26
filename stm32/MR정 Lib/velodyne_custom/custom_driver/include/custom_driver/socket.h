#ifndef SOCKET_H
#define SOCKET_H

#include <sys/socket.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <linux/socket.h>
#include <netinet/tcp.h>

#include <arpa/inet.h>

#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <thread>
#include <chrono>

#include <iostream>
#include <string.h>

namespace  Network {

inline bool connectToTCPServer(int& client_sd, const std::string& addr, const int port)
{
    std::string t_error;

    //Set socket
    int _client_sd;
    if((_client_sd = socket(AF_INET, SOCK_STREAM, 0)) == -1)
    {
        t_error = "ERROR::connectToTCPServer(socket): ";
        t_error += strerror(errno);
        throw t_error;
    }

    //Set tcp non-blocking
    fcntl(_client_sd, F_SETFL, O_NONBLOCK);

    //Try connect to tcp server
    struct sockaddr_in sin;
    memset((char*) &sin, '\0', sizeof(sin));
    sin.sin_family = AF_INET;
    sin.sin_port = htons(port);
    sin.sin_addr.s_addr = inet_addr(addr.c_str());
    memset(&(sin.sin_zero), 0, 8);

    int result = connect(_client_sd, (struct sockaddr*)&sin, sizeof(sin));

    if(result == 0)
    {
        std::cout << "connected!" << std::endl;
    }
    else if (errno == EINPROGRESS)
    {
        int error = 0;
        socklen_t err_len = sizeof(error);

        fd_set myset;
        int valopt = 0;

        struct timeval tv;
        tv.tv_sec = 1;
        tv.tv_usec = 0;

        FD_ZERO(&myset);
        FD_SET(_client_sd, &myset);

        result = select(_client_sd+1, NULL, &myset, NULL, &tv);

        if (result < 0 && errno != EINTR)
        {
            t_error = "Error while connecting";
            throw t_error;
        }
        else if (result > 0)
        {
            // Socket selected for write
            err_len = sizeof(int);
            if (getsockopt(_client_sd, SOL_SOCKET, SO_ERROR, (void*)(&valopt), &err_len) < 0)
            {
                t_error = "Error: getsockopt";
                throw t_error;
            }

            // Check the value returned...
            if (valopt)
            {
                t_error = "Error: delay";
                throw t_error;
            }
        }
        else
        {
            t_error = "Error: Timeout";
            throw t_error;
        }
    }
    else
    {
        t_error = "connected fail";
        throw t_error;
    }

    //Unset tcp non-blocking
    int old_fcntl = fcntl(_client_sd, F_GETFL);
    if(old_fcntl == -1)
        perror("cannot get fcntl");
    fcntl(_client_sd, F_SETFL, old_fcntl & ~O_NONBLOCK);

    //Set tcp nodelay socket opt
    int flag = 1;
    if (setsockopt(_client_sd, IPPROTO_TCP, TCP_NODELAY, (const char*)&flag, sizeof(flag)))
    {
        t_error = "tcp nodelay error";
        throw t_error;
    }

    client_sd = _client_sd;
    return true;
}

inline bool createUDPServer(int& server_sd, const int port)
{
    std::string t_error;

    //Set socket
    int _server_sd;
    if((_server_sd = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
    {
        t_error = "ERROR::createUDPServer(socket): ";
        t_error += strerror(errno);
        throw t_error;
        //return false;
    }

    //Set timeout socket opt
    struct timeval tv;
    tv.tv_sec = 3;
    tv.tv_usec = 0;
    if (setsockopt(_server_sd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv)) < 0)
        perror("SO_RCVTIMEO error");

    //Set reuse address socket opt
    int reuseAddr = 1;
    int state = setsockopt(_server_sd, SOL_SOCKET, SO_REUSEADDR, (const char*)&reuseAddr, sizeof(reuseAddr));
    if(state)
        perror("SO_REUSEADDR error");

    //Try bind udp server
    struct sockaddr_in sin;
    memset((char*)&sin, '\0', sizeof(sin));
    sin.sin_family = AF_INET;
    sin.sin_port = htons(port);
    sin.sin_addr.s_addr = htonl(INADDR_ANY);
    memset(&(sin.sin_zero), 0, 8);

    if (bind(_server_sd, (struct sockaddr *)&sin, sizeof(struct sockaddr)))
    {
        close(_server_sd);
        t_error = "ERROR::createUDPServer(bind): ";
        t_error += strerror(errno);
        throw t_error;
        //return false;
    }

    server_sd = _server_sd;
    return true;
}

}
#endif // SOCKET_H
