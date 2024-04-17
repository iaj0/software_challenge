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
        // complete this code
        rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr client;
        rclcpp::TimerBase::SharedPtr timer;

        static const unsigned int NUMBER_OF_TURTLES{2};

        typedef struct turtle_info {
          float x_pos;
          float y_pos;
          float rad;
        } turtle_info;
  
        std::vector<std::string> turtle_names{"stationary_turtle", "moving_turtle"};
        std::vector<turtle_info> turtle_bio{{.x_pos = 5, .y_pos = 5, .rad = 0},
                                          {.x_pos = 25, .y_pos = 10, .rad = 0}};
  
        // map of turtle name to turtle information
        std::map<std::string, turtle_info> turtle_description;

        std::map<std::string, turtle_info>::iterator it = turtle_description.begin();


        void spawn_turtle();

};

} // namespace composition
