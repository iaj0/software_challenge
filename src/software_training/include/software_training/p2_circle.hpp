#pragma once

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace composition {

class p2_circle : public rclcpp::Node {
    public:
        p2_circle(const rclcpp::NodeOptions& options);

    private:
        // complete this code
        // publisher
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
        //callback timer
        rclcpp::TimerBase::SharedPtr timer;
        
        // linear and angular velocities
        float linear_x = 1.0;
        float linear_y = 1.0;
        float linear_z = 1.0;
        float angular_x = 1.0;
        float angular_y = 1.0;
        float angular_z = 1.0;
};


} // namespace composition