#include "../include/software_training/p6_waypoint.hpp"

namespace composition {

p6_waypoint:: p6_waypoint(const rclcpp::NodeOption& options) : Node{"p6_waypoint", options} {
    // idea: 
    this->publisher = this->create_publisher<geometry_msgs::msg::Twist>(
        "/moving_turtle/cmd_vel", 10
    );

    auto subscriber_callback = [this](const turtlesim::msg::Pose::SharedPtr msg) -> void {
        this->p6_waypoint::x = msg->x;
        this->p6_waypoint::y = msg->y;
        this->p6_waypoint::theta = msg->theta;
        this->p6_waypoint::linear_velocity = msg->linear_velocity;
        this->p6_waypoint::angular_velocity = msg->angular_velocity;
    };

    this->subscriber = this->create_subscription<turtlesim::msg::Pose>(
        "/moving_turtle/pose", 10
    );

    this->action_server = 
        rclcpp_action::create_server<software_training::action::Waypoint>(
            this, "p6_waypoint_action_server",
            std::bind(&p6_waypoint::handle_goal, this, _1, _2),
            std::bind(&p6_waypoint::handle_cancel, this, _1),
            std::bind(&p6_waypoint::handle_accepted, this, _1)
        );
}

rclcpp_action::GoalResponse p6_waypoint::handle_goal(
    const rclcpp_action::GoalUUID &uuid, 
    std::shared_ptr<const software_training::action::Waypoint::Goal> goal) {

    
}

} // namespace composition