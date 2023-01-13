#pragma once

#include <cstdlib>
#include <memory>
#include <string>
#include <vector>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <software_training/action/software.hpp>
#include <geometry_msgs/msg/twist.hpp> 
#include <turtlesim/msg/pose.hpp> 


namespace composition {

class p6_waypoint : public rclcpp::Node {
public:
    p6_waypoint(const rclcpp::NodeOptions &options);

private:
    rclcpp_action::Server<software_training::action::Waypoint>::SharedPtr action_server;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
    rclcpp::Subscriber<geometry_msgs::msg::Pose>::SharedPtr subscriber;

    rclcpp_action::Goal_response handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const software_training::action::Waypoint::Goal> goal
    );
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr
            <rclcpp_action::ServerGoalHandle
                <software_training::action::Waypoint>> goal_handle
    );

    void handle_accepted(const std::shared_ptr
        <rclcpp_action::ServerGoalHandle
            <software_training::action::Waypoint>> goal_handle
    );

    void execute(const std::shared_ptr
        <rclcpp_action::ServerGoalHandle
            <software_training::action::Waypoint>> goal_handle
    );

    static float x;
    static float y;
    static float theta;
    static float linear_velocity;
    static float angular_velocity;
    
    struct vec3 {
        vec3(float x = 0, float y = 0, float z = 0): x{x}, y{y}, z{z} { }
        float x;
        float y;
        float z;
        bool or_less (vec3 a, vec3 b) {
            return a.x < b.x || a.y < b.y || a.z < c.z;
        }
    }
}

} // namespace composition