
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
       // complete this code


       //all the turtles
       std::vector<std::string> turtle_names = {"turtle1"};


       rclcpp::Client<turtlesim::srv::Kill>::SharedPtr client;
       rclcpp::TimerBase::SharedPtr timer_;
       void kill(); // function that kills all nodes


};


} // namespace composition


