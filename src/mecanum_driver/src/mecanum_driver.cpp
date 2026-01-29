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
    
    // create a can communication port 
    // create a robot consisting of 4 mecanum wheels

    // CanComm* can_comm;
    // MecanumRobot* robot;

    std::unique_ptr<CanComm> can_comm_;

    std::unique_ptr<ODriveMotor> fl_;
    std::unique_ptr<ODriveMotor> fr_; 
    std::unique_ptr<ODriveMotor> rl_; 
    std::unique_ptr<ODriveMotor> rr_;

    std::unique_ptr<MecanumRobot> robot_;

    // 4 mecanum wheels (motor) here
    // ODriveMotor* fl; // front left
    // ODriveMotor* fr; // front right
    // ODriveMotor* rl; // rear left
    // ODriveMotor* rr; // rear right

    public:
    MecanumDriver()
    : Node("mecanum_driver_node")
    {
        // can_comm = new CanComm();
        can_comm_ = std::make_unique<CanComm>();
        can_comm_->init("vcan0");

        fl_ = std::make_unique<ODriveMotor>(0, can_comm_.get());
        fr_ = std::make_unique<ODriveMotor>(1, can_comm_.get());
        rl_ = std::make_unique<ODriveMotor>(2, can_comm_.get());
        rr_ = std::make_unique<ODriveMotor>(3, can_comm_.get());

        robot_ = std::make_unique<MecanumRobot>(fl_.get(), fr_.get(), rl_.get(), rr_.get());
        // allocate 4 new odrive wheels with different ids
        // fl = new ODriveMotor(0, can_comm);
        // fr = new ODriveMotor(1, can_comm);
        // rl = new ODriveMotor(2, can_comm);
        // rr = new ODriveMotor(3, can_comm);

        // robot = new MecanumRobot(fl, fr, rl, rr);
        

        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&MecanumDriver::robot_callback, this, _1));
        RCLCPP_INFO(this->get_logger(), "the mecanum node started ok!\n");
    };
        
    ~MecanumDriver(){
        if(fl_) fl_->setAxisState(OdriveAxisState::IDLE);
        if(fr_) fr_->setAxisState(OdriveAxisState::IDLE);
        if(rl_) rl_->setAxisState(OdriveAxisState::IDLE);
        if(rr_) rr_->setAxisState(OdriveAxisState::IDLE);

        // delete robot;
        // delete fl; delete fr; delete rl; delete rr;
        // delete can_comm;
    }
    
    void robot_callback(const geometry_msgs::msg::Twist & msg)
    {
        float vx = msg.linear.x;
        float vy = msg.linear.y;
        float vz = msg.angular.z;

        RCLCPP_INFO(this->get_logger(), "Received: vx = %.2f, vy = %.2f, vz     = %.2f", vx, vy, vz);

        if (robot_!=nullptr) {
        robot_->drive(vx, vy, vz);
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
