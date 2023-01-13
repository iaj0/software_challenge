#include "../include/software_training/p6_waypoint.hpp"

namespace composition {

p6_waypoint:: p6_waypoint(const rclcpp::NodeOption& options) : Node{"p6_waypoint", options} {
    // idea: 
    this->publisher = this->create_publisher<geometry_msgs::msg::Twist>(
        "/moving_turtle/cmd_vel", 10
    );

    auto subscriber_callback = [this](const turtlesim::msg::Pose::SharedPtr msg) -> void {
        this->p6_waypoint::x = msg->x;
        this->p6_waypoint::y = msg->y;
        this->p6_waypoint::theta = msg->theta;
        this->p6_waypoint::linear_velocity = msg->linear_velocity;
        this->p6_waypoint::angular_velocity = msg->angular_velocity;
    };

    this->subscriber = this->create_subscription<turtlesim::msg::Pose>(
        "/moving_turtle/pose", 10
    );

    this->action_server = 
        rclcpp_action::create_server<software_training::action::Waypoint>(
            this, "p6_waypoint_action_server",
            std::bind(&p6_waypoint::handle_goal, this, _1, _2),
            std::bind(&p6_waypoint::handle_cancel, this, _1),
            std::bind(&p6_waypoint::handle_accepted, this, _1)
        );
}

rclcpp_action::GoalResponse p6_waypoint::handle_goal(
    const rclcpp_action::GoalUUID &uuid, 
    std::shared_ptr<const software_training::action::Waypoint::Goal> goal) {
    
    std::cout<<"HANDLE_GOAL()\n";
    std::cout<<"linear: ";
    std::cout<<goal->linear_pos.x<<" ";
    std::cout<<goal->linear_pos.y<<" ";
    std::cout<<goal->linear_pos.z<<" ";
    std::cout<<"angular: ";
    std::cout<<goal->angular_pos.x<<" ";
    std::cout<<goal->angular_pos.y<<" ";
    std::cout<<goal->angular_pos.z<<" ";
    std::cout<<std::endl;

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp::CancelResponse p6_waypoint:: handle_cancel(
    const std::shared_ptr<GoalHandleActionServer> goal_handle) {
    
    std::cout<<"CANCEL_GOAL()"<<std::endl;
    
    return rclcpp_action::CancelResponse::ACCEPT;
}

void p6_waypoint::handle_accepted(
    const std::shared_ptr<GoalHandleActionServer> goal_handle) {
    
    std::thread{
        std::bind(&p6_waypoint::execute, this, _1),
        goal_handle
    }.detach();
}

void p6_waypoint:: execute(
    const std::shared_ptr<GoalHandleActionServer> goal_handle) {


    const auto goal = goal_handle->get_goal();
    auto start_time = this->now();
    p6_waypoint::vec3 cur_lin{};
    p6_waypoint::vec3 cur_ang{}; 

    p6_waypoint::vec3 goal_lin{goal->linear_pos.x, goal->linear_pos.y, goal->linear_pos.z};
    p6_waypoint::vec3 goal_ang{goal->angular_pos.x, goal->angular_pos.y, goal->angular_pos.z};

    while (rclcpp::ok() && (cur_lin.x < goal->linear_pos.x || cur_lin.y < goal->linear_pos.y || cur_lin.z < goal->) 

    auto feedback = std::make_shared<software_training::Pose>();

}



} // namespace composition