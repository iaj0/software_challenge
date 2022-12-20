// #ifndef P1_CLEAR_HPP
// #define P1_CLEAR_HPP
#pragma once

#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <turtlesim/srv/kill.hpp>


class p1_clear : public rclcpp::Node {
    public:
        p1_clear();
        int x;
};

// #endif