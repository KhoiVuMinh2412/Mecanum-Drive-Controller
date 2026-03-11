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
        RCLCPP_INFO(this->get_logger(), "Keyboard control is ready.\n"
                                      "---------------------------\n"
                                      "Moving around:\n"
                                      "   q    w    e\n"
                                      "   a    s    d\n"
                                      "   z    x    c\n\n"
                                      "j/l : rotate left/right\n"
                                      "space/k : emergency stop\n"
                                      "CTRL-C to quit\n"
                                      "---------------------------");

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
        double x_inc = 0.0;
        double y_inc = 0.0;
        double z_inc = 0.0;

        switch (c)
        {
        case 'w':
            x_inc = step;
            break;
        case 'q':
            x_inc = step;
            y_inc = step;
            break;
        case 'e':
            x_inc = step;
            y_inc = -step;
            break;
        case 'a':
            y_inc = step;
            break;
        case 's':
        case 'x':
            x_inc = -step;
            break;
        case 'd':
            y_inc = -step;
            break;
        case 'z':
            y_inc = step;
            x_inc = -step;
            break;
        case 'c':
            y_inc = -step;
            x_inc = -step;
            break;
        case ' ':
        case 'k':
            current_vx = 0.0;
            current_vy = 0.0;
            current_vz = 0.0;
            break;
        case 'j':
            z_inc = step;
            break;
        case 'l':
            z_inc = -step;
            break;
        case '\x03':
            RCLCPP_INFO(this->get_logger(), "Exiting on Ctrl+C");
            return;
        default:
            dirty = false;
            break;
        }

        // Normalize diagonal movement to maintain constant speed
        if (x_inc != 0.0 && y_inc != 0.0) {
            x_inc /= std::sqrt(2.0);
            y_inc /= std::sqrt(2.0);
        }

        current_vx += x_inc;
        current_vy += y_inc;
        current_vz += z_inc;

        if (dirty) {
            current_vx = std::clamp(current_vx, -max_speed, max_speed);
            current_vy = std::clamp(current_vy, -max_speed, max_speed);
            current_vz = std::clamp(current_vz, -max_speed, max_speed);

            c_message.linear.x = current_vx;
            c_message.linear.y = current_vy;
            c_message.angular.z = current_vz;
        }

        RCLCPP_INFO(this->get_logger(), "Publishing: vx=%.2f, vy=%.2f, vaz=%.2f", c_message.linear.x, c_message.linear.y, c_message.angular.z);
        publisher_->publish(c_message);
        }
    }

    private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::make_shared<KeyboardControl>()->main_callback();
    rclcpp::shutdown();
    return 0;
}
