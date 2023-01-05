#pragma once

#include <cstdlib>
#include <functional>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <software_training/srv/reset.hpp>
#include <turtlesim/srv/teleport_absolute.hpp>

namespace composition {
    
class p4_reset : public rclcpp::Node{
public:
    p4_reset(const rclcpp::NodeOptions &options);

private:
    rclcpp::Service<software_training::srv::Reset>::SharedPtr service;
    rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr client;
    void service_callback(
        const std::shared_ptr<software_training::srv::Reset::Request> request,
        std::shared_ptr<software_training::srv::Reset::Response> response);

    typedef struct turtle_position {
        turtle_position (float x, float y, float theta): 
            x{x}, y{y}, theta{theta} {}
        float x;
        float y;
        float theta; // rotation?
    } turtle;

    std::vector<turtle> turtles {{1,1,90}};

    

};

} // namespace composition