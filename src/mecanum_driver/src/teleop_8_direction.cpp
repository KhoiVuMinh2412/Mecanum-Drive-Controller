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
        RCLCPP_INFO(this->get_logger(), "Keyboard is ready, press K to brake,, Ctrl + C to break, WASD to control and J, L to turn\n");

        while (rclcpp::ok()) 
        {
        char c = getch();
        auto c_message = geometry_msgs::msg::Twist();

        switch (c)
        {
        case 'w':
            c_message.linear.x = speed;
            break;
        case 'q':
            c_message.linear.x = speed;
            c_message.linear.y = speed;
            break;
        case 'e':
            c_message.linear.x = speed;
            c_message.linear.y = -speed;
            break;
        case 'a':
            c_message.linear.y = speed;
            break;
        case 's':
            c_message.linear.x = -speed;
            break;
        case 'd':
            c_message.linear.y = -speed;
            break;
        case 'z':
            c_message.linear.y = speed;
            c_message.linear.x = -speed;
            break;
        case 'c':
            c_message.linear.y = -speed;
            c_message.linear.x = -speed;
            break;
        case 'k':
            c_message.linear.x = 0;
            c_message.linear.y = 0;
            break;
        case 'j':
            c_message.angular.z = turn;
            break;
        case 'l':
            c_message.angular.z = -turn;
            break;
        case '\x03':
            break;

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
