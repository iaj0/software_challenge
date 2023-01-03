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
        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;

        static constexpr float linear_x = 11;
        static constexpr float linear_y = 11;
        static constexpr float linear_z = 11;
        static constexpr float angular_x = 0.41;
        static constexpr float angular_y = 0.41;
        static constexpr float angular_z = 0.41;

};


} // namespace composition