// can interface with predefined struct(s)
#include <linux/can.h> 
#include <linux/can/raw.h>
#include "can_comm.hpp"
#include <unistd.h>


// linux socket group include
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

// in out proceeddings and string 
#include <iostream>
#include <string.h>

CanComm::CanComm() : socket_fd(-1), is_connected(false) {}; // constructor

CanComm::~CanComm() { // destructor
    if (is_connected){
        close(socket_fd); // closing the right way
    }
};

// init can interface 
bool CanComm::init(const std::string& interface_name) {
    if ((socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) // error handler
    {
        printf("failed to create socket\n");
        return false;
    }

    strcpy(ifr.ifr_name, interface_name.c_str()); // copy interface name to the interface name in ifr to find the index of the virtual can device
    if (ioctl(socket_fd, SIOCGIFINDEX, &ifr) < 0){
        printf("locate instance failes\n");
        return false;
    }

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(socket_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        printf("failed to bind");
        return false;
    }
    is_connected = true;
    std::cout << "Can interface " << interface_name << " created sucessfully!\n";
    return true;
};

bool CanComm::send_frame(int can_id, uint8_t* data, int len){ // send frame function
    if (!is_connected) return false; // safety check
    struct can_frame frame; // define the can_frame struct inherit from the struct in can.h
    frame.can_id = can_id;
    frame.can_dlc = len;
    memcpy(frame.data, data, len); // copy the data to the frame
    if (write(socket_fd, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)){
        printf("failed to write");
        return false;
    }
    return true;
};

bool CanComm::receive_frame(struct can_frame& frame){

    int nbytes = read(socket_fd, &frame, sizeof(struct can_frame));
    if (nbytes < 0) {
        return false;
    } 
    if (nbytes < (int)sizeof(struct can_frame)){
        return false;   
    }
    return true;
}

