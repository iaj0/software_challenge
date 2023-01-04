#pragma once

#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <turtlesim/srv/spawn.hpp>

namespace composition {

class p3_spawn : public rclcpp::Node {
    public:
        p3_spawn(const rclcpp::NodeOptions &options);
    private:

        typedef struct spawn_turtle {
            spawn_turtle(float x, float y, std::string name): 
                x{x}, y{y}, name{name} {}
            float x;
            float y;
            float theta = 0; // rotation?
            std::string name;
        } turtle;

        rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr client;
        std::vector<turtle> nodes = {
            {5, 5, "stationary_turtle"}, 
            {25, 10, "moving_turtle"}};
        rclcpp::TimerBase::SharedPtr timer;
};

} // namespace composition
