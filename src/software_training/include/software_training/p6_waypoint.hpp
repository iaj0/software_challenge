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

    using GoalHandleActionServer = rclcpp_action::ServerGoalHandle<software_training::action::Waypoint>;

private:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const software_training::action::Waypoint::Goal> goal
    );

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleActionServer> goal_handle
    );

    void handle_accepted(
        const std::shared_ptr<GoalHandleActionServer> goal_handle
    );

    void execute(const std::shared_ptr<GoalHandleActionServer> goal_handle);

    rclcpp_action::Server<software_training::action::Waypoint>::SharedPtr action_server;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;

    double x = 0.0f;
    double y = 0.0f;
    double theta = 0.0f;
    double linear_velocity = 0.0f;
    double angular_velocity = 0.0f;

    template <typename T>
 
    struct vec2 {
        vec2(T x=0, T y=0): x{x}, y{y} { }
        // copy constructor
        vec2(const vec2& a) {
            this->x = a.x;
            this->y = a.y;
        }
        // converting from types
        template <typename T2> 
        vec2(const vec2<T2> &other) {
            this->x = other.x;
            this->y = other.y;
        } 

        vec2 operator+(vec2& b) {
            return {this->x + b.x, this->y + b.y};
        }

        vec2 operator-(vec2&b) {
            return {this->x - b.x, this->y - b.y};
        }


        
        T x;
        T y;
        // current is within error of goal
        bool in_range(vec2 goal, double error) {
            return (abs(goal.x - x) <= error) && (abs(goal.y - y) <= error);
        }
    };

    typedef vec2<double> vec2d; 
    typedef vec2<double&> vec2d_ref;
};

} // namespace composition