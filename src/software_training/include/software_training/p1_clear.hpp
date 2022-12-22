// #ifndef P1_CLEAR_HPP
// #define P1_CLEAR_HPP
#pragma once

#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <turtlesim/srv/kill.hpp>

namespace composition {
class p1_clear : public rclcpp::Node {
    public:
        p1_clear(const rclcpp::NodeOptions &options);
    private:
        std::shared_ptr<rclcpp::Client<turtlesim::srv::Kill>> client;
        std::vector<std::string> nodes = {"turtle1"}; // why static?
        std::shared_ptr<rclcpp::TimerBase> timer_;
        void kill();

};

}

// #endif