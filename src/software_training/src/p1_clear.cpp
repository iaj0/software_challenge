#include "../include/software_training/p1_clear.hpp"
// #include <iostream>

// using namespace std::chrono_literals;
namespace software_training {

p1_clear::p1_clear(const rclcpp::NodeOptions &options) : Node("p1_clear", options){
    // why pass options?
    client = this->create_client<turtlesim::srv::Kill>("kill");
    // create_client<service type>(name)
    timer_ = create_wall_timer(
        std::chrono::duration
            <int, std::chrono::seconds::period>(1), 
        std::bind(&p1_clear::kill, this)
        // bind "this" to the first (only param) of kill()
    );
    
    std::cout<<"whast"<<std::endl;
}

void p1_clear::kill() {

    
    for (auto turtle : nodes) {
        auto request = std::make_shared<turtlesim::srv::Kill::Request>();
        auto callback = [this](rclcpp::Client<turtlesim::srv::Kill>::SharedFuture response) -> void {

            std::cout<<"finished"<<std::endl;
            rclcpp::shutdown();
        };
        
        auto result = client->async_send_request(request, callback);
    }

}

// int main() {
//     std::cout<<"HJELLOOO"<<std::endl;
//     return 0;
// }

}

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(software_training::p1_clear)