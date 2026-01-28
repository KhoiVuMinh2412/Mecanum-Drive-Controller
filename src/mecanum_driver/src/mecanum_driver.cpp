#include "mecanum.hpp"
#include "odrivemotor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>

using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace std;

class MecanumDriver : public rclcpp::Node {
    private:
    
    CanComm* can_comm;
    MecanumRobot* robot;

    ODriveMotor* fl;
    ODriveMotor* fr;
    ODriveMotor* rl;
    ODriveMotor* rr;

    public:
    MecanumDriver()
    : Node("mecanum_driver_node")
    {
        can_comm = new CanComm();
        can_comm->init("vcan0");

        fl = new ODriveMotor(0, can_comm);
        fr = new ODriveMotor(1, can_comm);
        rl = new ODriveMotor(2, can_comm);
        rr = new ODriveMotor(3, can_comm);

        robot = new MecanumRobot(fl, fr, rl, rr);
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&MecanumDriver::robot_callback, this, _1));
        RCLCPP_INFO(this->get_logger(), "the mecanum node started ok!\n");
    };
        
    ~MecanumDriver(){
        if(fl) fl->setAxisState(OdriveAxisState::IDLE);
        if(fr) fr->setAxisState(OdriveAxisState::IDLE);
        if(rl) rl->setAxisState(OdriveAxisState::IDLE);
        if(rr) rr->setAxisState(OdriveAxisState::IDLE);

        delete robot;
        delete fl; delete fr; delete rl; delete rr;
        delete can_comm;
    }
    
    void robot_callback(const geometry_msgs::msg::Twist & msg)
    {
        float vx = msg.linear.x;
        float vy = msg.linear.y;
        float vz = msg.angular.z;

        RCLCPP_INFO(this->get_logger(), "nhan lenh: vx = %.2f, vy = %.2f, vz     = %.2f", vx, vy, vz);

        if (robot!=nullptr) {
        robot->drive(vx, vy, vz);
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MecanumDriver>());
    rclcpp::shutdown();
    return 0;
}
