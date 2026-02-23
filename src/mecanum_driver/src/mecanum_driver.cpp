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

    CanComm* can_comm;
    MecanumRobot* robot;


    // 4 mecanum wheels (motor) here
    ODriveMotor* fl; // front left
    ODriveMotor* fr; // front right
    ODriveMotor* rl; // rear left
    ODriveMotor* rr; // rear right

    rclcpp::TimerBase::SharedPtr feedback_timer; // timer for listening to odrive feedback

    public:
    MecanumDriver()
    : Node("mecanum_driver_node")
    {
        can_comm = new CanComm();
        if (!can_comm->init("vcan0")) 
        {
            RCLCPP_ERROR(this->get_logger(), "Khong the mo cong can vcan0~\n");
        }
        // allocate 4 new odrive wheels with different ids
        fl = new ODriveMotor(0, can_comm);
        fr = new ODriveMotor(1, can_comm);
        rl = new ODriveMotor(2, can_comm);
        rr = new ODriveMotor(3, can_comm);

        robot = new MecanumRobot(fl, fr, rl, rr);

        feedback_timer = this->create_wall_timer(5ms, std::bind(&MecanumDriver::read_can_loop, this));
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


    void read_can_loop() 
    {
        struct can_frame frame;

        while (can_comm->receive_frame(frame))
        {
            fl->parseCanMessage(frame.can_id, frame.data, frame.can_dlc);
            fr->parseCanMessage(frame.can_id, frame.data, frame.can_dlc);
            rl->parseCanMessage(frame.can_id, frame.data, frame.can_dlc);
            rr->parseCanMessage(frame.can_id, frame.data, frame.can_dlc);
            
        }
        
    }


    void robot_callback(const geometry_msgs::msg::Twist & msg)
    {
        float vx = msg.linear.x;
        float vy = msg.linear.y;
        float vz = msg.angular.z;

        RCLCPP_INFO(this->get_logger(), "Received: vx = %.2f, vy = %.2f, vz     = %.2f", vx, vy, vz);

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
