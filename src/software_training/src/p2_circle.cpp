#include "../include/software_training/p2_circle.hpp"
#include  <iostream>


using namespace std::chrono_literals;
namespace composition {

p2_circle::p2_circle(const rclcpp::NodeOptions& options) 
    : Node{"p2_circle", options} {
    // complete this code
    this->publisher = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    //create publisher callback
    auto publisher_callback = [this](void) -> void {
        auto message = std::make_unique<geometry_msgs::msg::Twist>();
        message->linear.x = linear_x;
        message->linear.y = linear_y;
        message->linear.z = linear_z;
        message->angular.x = angular_x;
        message->angular.y = angular_y;  
        message->angular.z = angular_z;
        this->publisher->publish(std::move(message));
    };
    //create timer
    timer = this->create_wall_timer(100ms, publisher_callback);

}

} // namespace composition

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(composition::p2_circle)