#pragma once

#include <chrono>
#include <cstdlib>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <software_training/action/waypoint.hpp>
#include <geometry_msgs/msg/twist.hpp> 
#include <turtlesim/msg/pose.hpp> 


namespace composition {

class p6_waypoint : public rclcpp::Node {
public:
    p6_waypoint(const rclcpp::NodeOptions &options);

private:
    rclcpp_action::Server<software_training::action::Waypoint>::SharedPtr action_server;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber;

    rclcpp_action::GoalResponse handle_goal(
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
    
    template <typename T>
 
    struct vec3 {
        vec3(T x=0, T y=0, T z=0): x{x}, y{y}, z{z} { }
        T x;
        T y;
        T z;
        bool or_less(vec3 b) {
            return x < b.x || y < b.y || z < b.z;
        }
        vec3 advance(vec3& goal) {
            vec3 ans;
            ans.x = (x < goal.x) ? ++x : x;
            ans.y = (y < goal.y) ? ++y : y;
            ans.z = (z < goal.z) ? ++z : z;
            return ans;
        }
    };

    typedef vec3<float> vec3f; 
    typedef vec3<double> vec3d;
    typedef vec3<float&> vec3f_ref;
};

} // namespace composition