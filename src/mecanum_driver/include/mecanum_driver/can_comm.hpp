#ifndef CAN_COMM_HPP
#define CAN_COMM_HPP

#include <linux/can.h>
#include <linux/can/raw.h> // for dlc (data length code)
#include <iostream>
// #include <vector> 
// for using std::vector in send_frame
#include <array>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>


class CanComm {
    private:
    int socket_fd; // file descriptor
    struct sockaddr_can addr;
    struct ifreq ifr; // use to find index of the (virtual) can device
    bool is_connected; // Check whether the socket is connected

    public:
    CanComm();

    ~CanComm();

    bool init(const std::string& interface_name); // init the can interface

    bool send_frame(int can_id, uint8_t* data, int len);

    bool receive_frame(struct can_frame& frame);

};


#endif