#pragma once

#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.h>

namespace composition {

class p2_circle : public rclcpp::Node {
    public:
        p2_circle(const rclcpp::NodeOptions& options);
    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

        constexpr float linear_x = 11;
        constexpr float linear_y = 11;
        constexpr float linear_z = 11;
        constexpr float angular_x = 0.41;
        constexpr float angular_y = 0.41;
        constexpr float angular_z = 0.41;

        void rotate();
        void timer_callback();
            
            
}


} // namespace composition