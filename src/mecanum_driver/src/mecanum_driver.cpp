#include "mecanum.hpp"
#include "odrivemotor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <map>

using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace std;

class MecanumDriver : public rclcpp::Node {
    private:
    
    // Use smart pointers for automatic memory management and a map for easy lookup
    std::unique_ptr<CanComm> can_comm_;
    std::unique_ptr<MecanumRobot> robot_;
    std::map<int, std::unique_ptr<ODriveMotor>> motors_; // Motors map

    rclcpp::TimerBase::SharedPtr feedback_timer; // timer for listening to odrive feedback

    public:
    MecanumDriver()
    : Node("mecanum_driver")
    {
        // Declare and get parameters
        this->declare_parameter<std::string>("can_interface", "vcan0");
        this->declare_parameter<std::vector<int64_t>>("motor_ids", {0, 1, 2, 3});
        this->declare_parameter<double>("wheel_base", 0.4);
        this->declare_parameter<double>("track_width", 0.3);
        this->declare_parameter<double>("wheel_radius", 0.05);

        auto can_interface = this->get_parameter("can_interface").as_string(); // string type
        auto motor_ids_long = this->get_parameter("motor_ids").as_integer_array(); // array type
        
        auto wheel_base = this->get_parameter("wheel_base").as_double(); // double type
        auto track_width = this->get_parameter("track_width").as_double();
        auto wheel_radius = this->get_parameter("wheel_radius").as_double();

        can_comm_ = std::make_unique<CanComm>();
        if (!can_comm_->init(can_interface)) 
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CAN interface '%s'", can_interface.c_str());
            rclcpp::shutdown();
            return;
        }

        if (motor_ids_long.size() != 4) {
            RCLCPP_ERROR(this->get_logger(), "Expected 4 motor IDs, but got %zu", motor_ids_long.size());
            rclcpp::shutdown();
            return;
        }

        // Create motors and robot using the motors as the map for easy lookup
        motors_[motor_ids_long[0]] = std::make_unique<ODriveMotor>(motor_ids_long[0], can_comm_.get());
        motors_[motor_ids_long[1]] = std::make_unique<ODriveMotor>(motor_ids_long[1], can_comm_.get());
        motors_[motor_ids_long[2]] = std::make_unique<ODriveMotor>(motor_ids_long[2], can_comm_.get());
        motors_[motor_ids_long[3]] = std::make_unique<ODriveMotor>(motor_ids_long[3], can_comm_.get());

        for (auto& [id, motor] : motors_) 
        {
            motor->clearErrors();
            motor->setAxisState(OdriveAxisState::CLOSED_LOOP_CONTROL);
            motor->setControllerMode(ODriveControlMode::VEL_CONTROL);
        }

        robot_ = std::make_unique<MecanumRobot>(motors_[motor_ids_long[0]].get(), motors_[motor_ids_long[1]].get(), motors_[motor_ids_long[2]].get(), motors_[motor_ids_long[3]].get(), wheel_radius, wheel_base, track_width);

        feedback_timer = this->create_wall_timer(5ms, std::bind(&MecanumDriver::read_can_loop, this));
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&MecanumDriver::robot_callback, this, _1));
        RCLCPP_INFO(this->get_logger(), "Mecanum driver node started successfully!");
    };
        
    ~MecanumDriver(){
        for (auto const& [id, motor] : motors_) {
            if (motor) motor->setAxisState(OdriveAxisState::IDLE);
        }
        RCLCPP_INFO(this->get_logger(), "Mecanum driver node shutting down.");
    }


    void read_can_loop() 
    {
        struct can_frame frame;

        while (can_comm_->receive_frame(frame))
        {
            int node_id = (frame.can_id >> 5);
            if (motors_.count(node_id)) {
                motors_[node_id]->parseCanMessage(frame.can_id, frame.data);
            }
        }
        
    }


    void robot_callback(const geometry_msgs::msg::Twist & msg) // listens to topic on /cmd_vel
    {
        float vx = msg.linear.x;
        float vy = msg.linear.y;
        float vz = msg.angular.z;
        
        // This log is only activated when setting the log level in ros args to debug
        // Default, the log level will be INFO
        RCLCPP_DEBUG(this->get_logger(), "Received: vx = %.2f, vy = %.2f, vz = %.2f", vx, vy, vz);

        if (robot_ != nullptr) {
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
