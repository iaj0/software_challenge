#include "../include/software_training/p1_clear.hpp"
// #include <iostream>

// using namespace std::chrono_literals;
// using namespace composition;

p1_clear::p1_clear(const rclcpp::NodeOptions &options) : Node("p1_clear"){
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

void p1_clear::kill() {

    rclcpp::Client<turtlesim::srv::Kill>::SharedFuture x;
    auto callback = [this](std::shared_future<rclcpp::Client<turtlesim::srv::Kill>> response) -> void {
        (void) response;

        std::cout << "finished" << std::endl;
        rclcpp::shutdown();
    };
    for (auto turtle : nodes) {

        auto request = std::make_shared<turtlesim::srv::Kill::Request>();
        
        auto result = client->async_send_request(request, callback);
    }

}

// int main() {
//     std::cout<<"HJELLOOO"<<std::endl;
//     return 0;
// }

