#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>

using namespace std::chrono_literals;
using namespace std;
using std::placeholders::_1;

// creating a method to read the input key in terminal without pressing enter
// alternative to conio.h on windows
int getch(void)
{
    struct termios oldattr, newattr;
    int ch;
    tcgetattr( STDIN_FILENO, &oldattr);
    newattr = oldattr;
    newattr.c_lflag &= ~( ICANON | ECHO );
    tcsetattr( STDIN_FILENO, TCSANOW, &newattr);
    ch = getchar();
    tcsetattr( STDIN_FILENO, TCSANOW, &oldattr);
    return ch; 
}


class KeyboardControl : public rclcpp::Node {
    public:

    KeyboardControl()
    : Node("keyboard_control_node") 
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
    }

    void main_callback()
    {
        RCLCPP_INFO(this->get_logger(), "Keyboard is ready, press K to brake, Ctrl + C to break, WASD to control and J, L to turn\n");

        double current_vx = 0.0;
        double current_vy = 0.0;
        double current_vz = 0.0;
        auto c_message = geometry_msgs::msg::Twist();

        const double step = 0.05;
        const double max_speed = 1.0;
        while (rclcpp::ok()) 
        {
        char c = getch();
        bool dirty = true;

        switch (c)
        {
        case 'w':
            current_vx += step;
            break;
        case 'q':
            current_vx += step;
            current_vy += step;
            break;
        case 'e':
            current_vx += step;
            current_vy -= step;
            break;
        case 'a':
            current_vy += step;
            break;
        case 's':
            current_vx -= step;
            break;
        case 'd':
            current_vy -= step;
            break;
        case 'z':
            current_vy += step;
            current_vx -= step;
            break;
        case 'c':
            current_vy -= step;
            current_vx -= step;
            break;
        case ' ':
        case 'k':
            current_vx = 0.0;
            current_vy = 0.0;
            current_vz = 0.0;
            break;
        case 'j':
            current_vz += step;
            break;
        case 'l':
            current_vz -= step;
            break;
        case '\x03':
            return;
        default:
            dirty = false;
            break;
        }

        if (dirty) {
            current_vx = std::clamp(current_vx, -max_speed, max_speed);
            current_vy = std::clamp(current_vy, -max_speed, max_speed);
            current_vz = std::clamp(current_vz, -max_speed, max_speed);

            c_message.linear.x = current_vx;
            c_message.linear.y = current_vy;
            c_message.angular.z = current_vz;
        }

        RCLCPP_INFO(this->get_logger(), "button pressed: '%c'", c);
        publisher_->publish(c_message);
        }
    }

    private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

    double speed = 0.5;
    double turn = 1.0;
    
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::make_shared<KeyboardControl>()->main_callback();
    rclcpp::shutdown();
    return 0;
}
