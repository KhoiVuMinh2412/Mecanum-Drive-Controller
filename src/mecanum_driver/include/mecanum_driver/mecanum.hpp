#ifndef MECANUM_HPP
#define MECANUM_HPP

#include <iostream>
#include <string.h>
#include "odrivemotor.hpp"

class MecanumRobot
{
private:
    ODriveMotor *fl;
    ODriveMotor *fr;
    ODriveMotor *rl;
    ODriveMotor *rr;

    float wheel_rad;
    float wheel_base;
    float track_width;

    float rotational_component_;
    float wheel_circumference_;

public:
    MecanumRobot(ODriveMotor *fl, ODriveMotor *fr, ODriveMotor *rl, ODriveMotor *rr, float wheel_rad, float wheel_base, float track_width);

    void drive(float linear_x, float linear_y, float angular_z);
};

#endif