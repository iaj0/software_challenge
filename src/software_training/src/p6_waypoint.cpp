#include "../include/software_training/p6_waypoint.hpp"
using namespace std::chrono_literals;
using namespace std::placeholders;
namespace composition {

p6_waypoint:: p6_waypoint(const rclcpp::NodeOptions& options) : Node{"p6_waypoint", options} {
        
    // idea: 
    // publisher to send moving data
    // subscriber to read current position
    // action server updates current pos and sends result + duration

    // (post) better way wouldve been to give p6_waypoint it's own vec2s instead of making curr_pos

    this->action_server = rclcpp_action::create_server<software_training::action::Waypoint>(
        this, "p6_waypoint",
        std::bind(&p6_waypoint::handle_goal, this, _1, _2),
        std::bind(&p6_waypoint::handle_cancel, this, _1),
        std::bind(&p6_waypoint::handle_accepted, this, _1)
    );

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
        "/moving_turtle/pose", 10, subscriber_callback
    );
}

rclcpp_action::GoalResponse p6_waypoint::handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const software_training::action::Waypoint::Goal> goal) {

    (void)uuid;
    // print
    RCLCPP_INFO(this->get_logger(), "linear x: %f, y: %f", goal->x, goal->y);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse p6_waypoint:: handle_cancel(
        const std::shared_ptr<GoalHandleActionServer> goal_handle) {
    (void) goal_handle;
    RCLCPP_INFO(this->get_logger(), "CANCEL");
    
    return rclcpp_action::CancelResponse::ACCEPT;
}

void p6_waypoint:: handle_accepted(const std::shared_ptr
        <rclcpp_action::ServerGoalHandle
            <software_training::action::Waypoint>> goal_handle) {
    
    // run execute on seperate thread (detach makes demon process)
    std::thread{
        std::bind(&p6_waypoint::execute, this, _1),
        goal_handle
    }.detach();
}

void p6_waypoint::execute(const std::shared_ptr<GoalHandleActionServer> goal_handle) {
    auto start_time = this->now(); 

    const auto goal = goal_handle->get_goal();
    auto result = std::make_unique<software_training::action::Waypoint::Result>();

    vec2d_ref curr_pos{this->p6_waypoint::x, this->p6_waypoint::y};
    vec2d old_pos {curr_pos};
    vec2d goal_pos{goal->x, goal->y};

    RCLCPP_INFO(this->get_logger(), "current position: (%f, %f)", curr_pos.x, curr_pos.y);
    RCLCPP_INFO(this->get_logger(), "goal: (%f, %f)", goal_pos.x, goal_pos.y);

    double error = 0.01; // lower error = takes longer to get exact position
                         // need error limit cuz decimal precision means its hard to get exact position
    while(rclcpp::ok() && !(goal_pos.in_range(curr_pos, error))) {
        // update velocity with current pos for higher accuracy
        vec2d velocity{(goal_pos.x - curr_pos.x) * 2, (goal_pos.y - curr_pos.y) * 2};
        RCLCPP_INFO(this->get_logger(), "velocity: (%f, %f)", velocity.x, velocity.y);

        auto message = std::make_unique<geometry_msgs::msg::Twist>();
        message->angular.x = 0.0f;
        message->angular.y = 0.0f;
        message->angular.z = 0.0f;
        message->linear.x = velocity.x;
        message->linear.y = velocity.y;
        message->linear.z = 0.0f;

        auto feedback = std::make_unique<software_training::action::Waypoint::Feedback>();
        feedback->distance = sqrt(pow(goal_pos.x - curr_pos.x, 2)+ pow(goal_pos.x - curr_pos.y, 2));

        this->publisher->publish(std::move(message));
        goal_handle->publish_feedback(std::move(feedback));
    }
    
    if (rclcpp::ok()) {

        auto message = std::make_unique<geometry_msgs::msg::Twist>();
        message->angular.x = 0.0f;
        message->angular.y = 0.0f;
        message->angular.z = 0.0f;
        message->linear.x = 0.0f;
        message->linear.y = 0.0f;
        message->linear.z = 0.0f;

        this->publisher->publish(std::move(message));

        rclcpp::Time end_time = this->now();       
        rclcpp::Duration duration = end_time - start_time; 

        result->duration = duration.nanoseconds();

        goal_handle->succeed(std::move(result)); 
        RCLCPP_INFO(this->get_logger(), "DONE!!!!");
        RCLCPP_INFO(this->get_logger(), "moved from (%f, %f) -> (%f, %f)", old_pos.x, old_pos.y, goal_pos.x, goal_pos.y);
    }
}

} // namespace composition

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(composition::p6_waypoint)