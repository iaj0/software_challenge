#include "../include/software_training/p2_circle.hpp"
#include <geometry_msgs/msg/twist.hpp>
namespace composition {

p2_circle::p2_circle(const rclcpp::NodeOptions& options) : Node{"p1_clear", options} {
    publisher = this->create_publisher<std_msgs::msg::String>
}

}