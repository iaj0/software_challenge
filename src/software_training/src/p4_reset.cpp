#include "../include/software_training/p4_reset.hpp"
#include <iostream>

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace composition {




p4_reset:: p4_reset(const rclcpp::NodeOptions& options) : Node{"p4_reset", options} {

    std::cout<<"\nSTART\n"<<std::endl;

    this->client = this->create_client<turtlesim::srv::TeleportAbsolute>("/moving_turtle/teleport_absolute");
    this->service = this->create_service<software_training::srv::Reset>(
        "/reset_moving_turtle", std::bind(&p4_reset::service_callback, this, _1, _2));


    auto client_request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
    
    client_request->x = this->turtles[0].x;
    client_request->y = this->turtles[0].y;
    client_request->theta = this->turtles[0].theta;

    std::cout<<client_request->theta<<std::endl;


    auto response_callback = [this](rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedFuture future) -> void {
        (void) future;
        std::cout<<"teleport done"<<std::endl;
    };

    auto result = client->async_send_request(std::move(client_request), response_callback);




    std::cout<<"\nEND\n"<<std::endl;
    
}

void p4_reset:: service_callback(
        const std::shared_ptr<software_training::srv::Reset::Request> request,
        std::shared_ptr<software_training::srv::Reset::Response> response) {
    
    std::cout<<"start callback"<<std::endl;

    (void) request; // its empty so avoid warning from compiler !?

    auto client_request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
    
    client_request->x = this->turtles[0].x;
    client_request->y = this->turtles[0].y;
    client_request->theta = this->turtles[0].theta;

    std::cout<<client_request->theta<<std::endl;


    auto response_callback = [this](rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedFuture future) -> void {
        (void) future;
        std::cout<<"teleport done"<<std::endl;
    };

    auto result = client->async_send_request(std::move(client_request), response_callback);

    response->result = true; // update request

}

} // namespace composition

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(composition::p4_reset)