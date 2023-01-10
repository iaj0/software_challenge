#include "../include/software_training/p5_distance.hpp"
#include <iostream>

using namespace std::chrono_literals;

namespace composition {

p5_distance:: p5_distance (const rclcpp::NodeOptions &options) : Node{"p5_distance", options} {

    auto stationary_callback = [this] (const turtlesim::msg::Pose::SharedPtr msg) -> void {
        this->stationary.x = msg->x;
        this->stationary.y = msg->y;
    };

    auto moving_callback = [this] (const turtlesim::msg::Pose::SharedPtr msg) -> void {
        this->moving.x = msg->x;
        this->moving.y = msg->y;
    };

    auto publish_callback = [this] (void) -> void {
        position distance;

        distance.x = {abs(this->stationary.x - this->moving.x)};
        distance.y = {abs(this->stationary.y - this->moving.y)};

        auto msg = std::make_unique<software_training::msg::Distance>();
        
        msg->dx = distance.x;
        msg->dy = distance.y;
        msg->distance = sqrt(pow(distance.x, 2) + pow(distance.y, 2));

        std::cout<<"stationary is "<<msg->dx<<" units away on x axis\n";
        std::cout<<"stationary is "<<msg->dy<<" units away on y axis\n";
        std::cout<<"total distance: "<<msg->distance<<std::endl;

        this->publisher->publish(std::move(msg));
    };

    callbacks = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    auto callback_option = rclcpp::SubscriptionOptions();
    callback_option.callback_group = callbacks;
    callback_option.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;

    std::string s_name{"/statistics"};
    std::string node_name{this->get_name()};

    std::string stat_name = s_name + node_name;
    callback_option.topic_stats_options.publish_topic = stat_name.c_str();

    stationary_sub = this->create_subscription<turtlesim::msg::Pose>(
        "/stationary_turtle/pose", 10, stationary_callback, callback_option
    ); // qos depth of 10 (what does that mean lol)

    moving_sub = this->create_subscription<turtlesim::msg::Pose>(
        "moving_turtle/pose",10, moving_callback,callback_option
    );

    publisher = this->create_publisher<software_training::msg::Distance>(
        "/difference", 10
    );

    timer = this->create_wall_timer(0.25s, publish_callback, callbacks);

}


} // namespace composition

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(composition::p5_distance)