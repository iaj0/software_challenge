#include "../include/software_training/p1_clear.hpp"
#include <iostream>

using namespace composition;

p1_clear::p1_clear(const rclcpp::NodeOptions &options) : Node("p1_clear", options){
    // why pass options?
    client = this->create_client<turtlesim::srv::Kill>("kill");
    // create_client<service type>(name)
    timer_ = create_wall_timer(
        std::chrono::duration
            <int, std::chrono::seconds::period>(1), 
        std::bind(&p1_clear::kill, this)
    );
    
    std::cout<<"whast"<<std::endl;
}


// int main() {
//     std::cout<<"HJELLOOO"<<std::endl;
//     return 0;
// }
