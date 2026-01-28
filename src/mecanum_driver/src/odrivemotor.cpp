#include "odrivemotor.hpp"
#include <iostream>
#include <string.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <array>

// constructor va destructor
ODriveMotor::ODriveMotor(int node_id, CanComm *can_comm)
{ // check dung thu tu parameters
    this->can_comm_ptr = can_comm;
    this->node_id_ = node_id;
};

// union FloatConverter {
//     float f_val; // float value
//     uint8_t fin_val[4]; // int_8 value
// };

ODriveMotor::~ODriveMotor() {}; // goi destructor

// ham de set trang thai axis
bool ODriveMotor::setAxisState(OdriveAxisState state)
{
    uint32_t can_id = (node_id_ << 5) | (uint8_t)OdriveCommandID::SET_AXIS_STATE; // can_id = (axis_id << 5) | cmd_id

    uint32_t state_val = (uint32_t)state; // ep kieu uint8 trong axisState thanh uint32

    uint8_t data[8] = {0};
    data[0] = state_val & 0xFF;         // tao mat na 0xFF de loc bit thua dang truoc
    data[1] = (state_val >> 8) & 0xFF;  // dich sang phai 4*2 bit
    data[2] = (state_val >> 16) & 0xFF; // dich sang phai 4*4 bit
    data[3] = (state_val >> 24) & 0xFF; // dich sang phai 4*6 bit

    return can_comm_ptr->send_frame(can_id, data, 4); // gui frame len can bus
};

// ham de gui lenh set van toc
bool ODriveMotor::setVelocity(float velocity)
{
    uint32_t can_id = (node_id_ << 5) | (uint8_t)OdriveCommandID::SET_INPUT_VEL;
    // FloatConverter converter;
    // converter.f_val = velocity;

    uint8_t data[8] = {0}; // 4 byte dau la vel, 4 byte sau la torque ff
    // data[0] = converter.fin_val[0];
    // data[1] = converter.fin_val[1];
    // data[2] = converter.fin_val[2];
    // data[3] = converter.fin_val[3];

    memcpy(data, &velocity, sizeof(float)); // dung duoc vi may tinh cung dang dung chuan ieee 754

    return can_comm_ptr->send_frame(can_id, data, 8);
};

// ham de clear error
// ham nay khong can data, chi gui null pointer
bool ODriveMotor::clearErrors()
{
    uint32_t can_id = (node_id_ << 5) | (uint8_t)OdriveCommandID::CLEAR_ERRORS;
    return can_comm_ptr->send_frame(can_id, nullptr, 0);
};

bool ODriveMotor::setPosition(float position)
{
    uint32_t can_id = (node_id_ << 5) | (uint8_t)OdriveCommandID::SET_INPUT_POS;
    uint8_t data[8] = {0};
    memcpy(data, &position, sizeof(float));
    return can_comm_ptr->send_frame(can_id, data, 8);
};

bool ODriveMotor::setControllerMode(ODriveControlMode mode)
{
    uint32_t can_id = (node_id_ << 5) | (uint8_t)OdriveCommandID::SET_CONTROLLER_MODE;

    int32_t control_mode = (int32_t)mode;
    int32_t input_mode = (int32_t)ODriveInputMode::PASSTHROUGH;

    uint8_t data[8] = {0};
    data[0] = control_mode         & 0xFF;
    data[1] = (control_mode >> 8)  & 0xFF;
    data[2] = (control_mode >> 16) & 0xFF;
    data[3] = (control_mode >> 24) & 0xFF;

    data[4] = input_mode & 0xFF;
    data[5] = (input_mode >> 8)    & 0xFF;
    data[6] = (input_mode >> 16)   & 0xFF;
    data[7] = (input_mode >> 24)   & 0xFF;

    return can_comm_ptr->send_frame(can_id, data, 8);
};

void ODriveMotor::parseCanMessage(uint32_t can_id, uint8_t* data, int len) {
    int mess_node_id = (can_id >> 5); // lay node id tu can_id

    if (mess_node_id != this->node_id_) // so sanh xem dung id cua motor khong
    {
        return;
    }
    
    int cmd_id = (can_id & 0x1F);

    switch (cmd_id)
    {
    case (int)OdriveCommandID::GET_ENCODER_ESTIMATES:
        memcpy(&this->pos_feedback, data, sizeof(float)); // doc 4 byte dau de lay pos
        memcpy(&this->vel_feedback, &data[4], sizeof(float)); // doc 4 byte sau de lay vel
        break;  
    
    case (int)OdriveCommandID::HEARTBEAT:
        memcpy(&this->axis_error, data, sizeof(uint32_t)); // 4 byte dau nhan axis error
        memcpy(&this->axis_current_state, data + 4, sizeof(uint32_t)); // 4 byte sau nhan ve axis current state
        break;
    }

}