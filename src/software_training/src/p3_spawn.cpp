#include "../include/software_training/p3_spawn.hpp"
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>
#include <iostream>

namespace composition {

p3_spawn:: p3_spawn(const rclcpp::NodeOptions &options)
    : Node{"p3_spawn", options} {
    // complete this code

    client = this->create_client<turtlesim::srv::Spawn>("spawn");
    //map turtle name to turtle information
    //stationary turtle
    turtle_description[turtle_names[0]] = turtle_bio[0];
    //moving turtle
    turtle_description[turtle_names[1]] = turtle_bio[1];

    p3_spawn::spawn_turtle();

} 

void p3_spawn::spawn_turtle(){
    for(auto turtles: turtle_names){
        auto message = std::make_shared<turtlesim::srv::Spawn::Request>();
        message->x = turtle_description[turtles].x_pos;
        message->y = turtle_description[turtles].y_pos;
        message->theta = turtle_description[turtles].rad;
        message->name = turtles;
        std::cout<<turtle_description[turtles].x_pos<<std::endl;

        auto callback = [this] (void) -> void {
               //output to log
            //    (void)response;
                RCLCPP_INFO(this->get_logger(), "Spawned Turtle");

               rclcpp::shutdown(); // kill this node
       };
        auto result = client->async_send_request(message);
       
    }
    
}
    

} // namespace composition

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(composition::p3_spawn)