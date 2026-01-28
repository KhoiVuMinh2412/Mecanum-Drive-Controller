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
    // {
    //     if ((socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) // error handler
    //     {
    //         printf("Failed to create socket");
    //         return false;
    //     }

    //     strcpy(ifr.ifr_name, interface_name.c_str());
    //     if (ioctl(socket_fd, SIOCGIFINDEX, &ifr) < 0){
    //         printf("locate instance failes");
    //         return false;
    //     }
    //     addr.can_family = AF_CAN;
    //     addr.can_ifindex = ifr.ifr_ifindex;
    //     if (bind(socket_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    //         printf("failed to bind");
    //         return false;
    //     }
    //     is_connected = true;
    //     std::cout << "Can interface" << interface_name << "created sucessfully!";
    //     return true;
    // }

    bool send_frame(int can_id, uint8_t* data, int len);
    // { // function to send the fking frame :)
    //     if (!is_connected) return false; // safety check
    //     struct can_frame frame; // define the can_frame struct inherit from the struct in can.h
    //     frame.can_id = can_id;
    //     frame.can_dlc = len;
    //     memcpy(frame.data, data, len); // copy the data to the frame
    //     if (write(socket_fd, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)){
    //         printf("failed to write");
    //         return false;
    //     }
    //     return true;
    // }

    bool receive_frame(struct can_frame& frame);
    // {
    //     can_frame& frame_copy = frame;
    //     if (read(socket_fd, &frame, sizeof(struct can_frame)) == -1){
    //         printf("read error!");
    //         return false;
    //     }
    // }
};


#endif