#ifndef ODRIVE_MOTOR_HPP
#define ODRIVE_MOTOR_HPP

#include "can_comm.hpp"
#include <iostream>
#include <string.h>
#include <cstdint>
#include <array>

enum class OdriveCommandID : uint8_t {
    HEARTBEAT                   = 0x001,
    SET_AXIS_STATE              = 0x007,
    GET_ENCODER_ESTIMATES       = 0x009,
    SET_INPUT_VEL               = 0x00D,
    SET_INPUT_POS               = 0x00C,
    SET_CONTROLLER_MODE         = 0x00B,
    CLEAR_ERRORS                = 0x018,
};

enum class OdriveAxisState : uint8_t {
    UNDEFINED                               = 0x0,
    IDLE                                    = 0x1,
    STARTUP_SEQUENCE                        = 0x2,
    FULL_CALIBRATION_SEQUENCE               = 0x3,
    MOTOR_CALIBRATION                       = 0x4,
    ENCODER_INDEX_SEARCH                    = 0x6,
    ENCODER_OFFSET_CALIBRATION              = 0x7,
    CLOSED_LOOP_CONTROL                     = 0x8,
    LOCKIN_SPIN                             = 0x9,
    ENCODER_DIR_FIND                        = 0xA,
    HOMING                                  = 0xB,
    ENCODER_HALL_POLARITY_CALIBRATION       = 0xC,
    ENCODER_HALL_PHASE_CALIBRATION          = 0xD,
};

enum class ODriveControlMode : uint8_t {
    VEL_CONTROL                             = 1,
    POS_CONTROL                             = 2,
};

enum class ODriveInputMode : uint8_t {
    INACTIVE                                = 0x0,
    PASSTHROUGH                             = 0x1,
    VEL_RAMP                                = 0x2,
    POS_FILTER                              = 0x3,
    MIX_CHANNELS                            = 0x4,
    TRAP_TRAJ                               = 0x5,
    TORQUE_RAMP                             = 0x6,
    MIRROR                                  = 0x7,
    TUNING                                  = 0x8,
};


class ODriveMotor
{
    private:
    CanComm* can_comm_ptr;
    int node_id_ = 0;

    public:

    // constructor va destructor
    ODriveMotor(int node_id, CanComm* can_comm);
    ~ODriveMotor();

    // ham khoi tao
    bool init();

    // cac ham de set axis, vel, pos, controller mode
    bool setAxisState(OdriveAxisState state);
    bool setVelocity(float velocity);
    bool setPosition(float position);
    bool setControllerMode(ODriveControlMode mode);


    bool getEncoderEstimate();
    bool clearErrors();
    
    // cac bien luu tru thong tin nhan ve
    float pos_feedback = 0.0f;
    float vel_feedback = 0.0f;
    uint32_t axis_error;
    uint32_t axis_current_state;
    void parseCanMessage(uint32_t can_id, uint8_t* data, int len); // ham de xu ly cac tin nhan vao
};


#endif