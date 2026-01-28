#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "mecanum.hpp"
#include "odrivemotor.hpp"
#include <iostream>
#include <math.h>
#include <numbers>

MecanumRobot::MecanumRobot(ODriveMotor* fl, ODriveMotor* fr, ODriveMotor* rl, ODriveMotor* rr) {
    this->fl = fl;
    this->fr = fr;
    this->rl = rl;
    this->rr = rr;

    this->wheel_rad = 0.05f; // radius of the wheel
    this->wheel_base = 0.4f; // Lx
    this->track_width = 0.3f; // Ly

};

void MecanumRobot::drive(float linear_x, float linear_y, float angular_z) 
{
    float sum =  wheel_base + track_width;

    float v_fl = linear_x - linear_y - (sum * angular_z);
    float v_fr = linear_x + linear_y + (sum * angular_z);
    float v_rl = linear_x + linear_y - (sum * angular_z);
    float v_rr = linear_x - linear_y + (sum * angular_z);
    
    float circum = 2 * M_PI * this->wheel_rad;

    float input_fr = v_fl / circum;
    float input_fl = v_fr / circum;
    float input_rl = v_rl / circum;
    float input_rr = v_rr / circum;

    this->fl->setVelocity(input_fl);
    this->fr->setVelocity(input_fr);
    this->rl->setVelocity(input_rl);
    this->rr->setVelocity(input_rr);
}

