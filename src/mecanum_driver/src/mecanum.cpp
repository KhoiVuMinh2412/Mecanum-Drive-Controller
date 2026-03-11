#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "mecanum.hpp"
#include "odrivemotor.hpp"
#include <iostream>
#include <math.h>
#include <numbers>

MecanumRobot::MecanumRobot(ODriveMotor* fl, ODriveMotor* fr, ODriveMotor* rl, ODriveMotor* rr, float wheel_rad, float wheel_base, float track_width) {
    this->fl = fl;
    this->fr = fr;
    this->rl = rl;
    this->rr = rr;
 
    this->wheel_rad = wheel_rad;
    this->wheel_base = wheel_base;
    this->track_width = track_width;

    // Calculate constant values once during initialization
    this->rotational_component_ = (wheel_base + track_width) / 2.0f;
    this->wheel_circumference_ = 2 * M_PI * this->wheel_rad;
};

void MecanumRobot::drive(float linear_x, float linear_y, float angular_z) 
{
    // Use the pre-calculated member variables
    float v_fl = linear_x - linear_y - (this->rotational_component_ * angular_z);
    float v_fr = linear_x + linear_y + (this->rotational_component_ * angular_z);
    float v_rl = linear_x + linear_y - (this->rotational_component_ * angular_z);
    float v_rr = linear_x - linear_y + (this->rotational_component_ * angular_z);
    
    // Convert linear wheel velocity (m/s) to rotational velocity (turns/s)
    float input_fr = v_fr / this->wheel_circumference_;
    float input_fl = v_fl / this->wheel_circumference_;
    float input_rl = v_rl / this->wheel_circumference_;
    float input_rr = v_rr / this->wheel_circumference_;

    this->fl->setVelocity(input_fl);
    this->fr->setVelocity(input_fr);
    this->rl->setVelocity(input_rl);
    this->rr->setVelocity(input_rr);
}
