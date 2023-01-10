#pragma once

#include <cstdlib>
#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include <rclcpp/rclcpp.hpp>

#include <turtlesim/msg/pose.hpp>
#include <software_training/msg/distance.hpp>
#include <rclcpp/rclcpp.hpp>

namespace composition {

class p5_distance : public rclcpp::Node {
public:
    p5_distance(const rclcpp::NodeOptions &options);
private:
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr stationary_sub;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr moving_sub;
    rclcpp::Publisher<software_training::msg::Distance>::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::CallbackGroup::SharedPtr callbacks;
    struct position {
        float x;
        float y;
    }; 
    position moving;
    position stationary;
    float distance;
};

} // namespace composition