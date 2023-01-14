#include "../include/software_training/p6_waypoint.hpp"
using namespace std::chrono_literals;
using namespace std::placeholders;
namespace composition {

p6_waypoint:: p6_waypoint(const rclcpp::NodeOptions& options) : Node{"p6_waypoint", options} {
    // idea: 
    // publisher to send moving data
    // subscriber to read current position
    // action server updates current pos and sends result + duration
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
    
    // print
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

rclcpp_action::CancelResponse p6_waypoint:: handle_cancel(
        const std::shared_ptr
            <rclcpp_action::ServerGoalHandle
                <software_training::action::Waypoint>> goal_handle) {
    
    std::cout<<"CANCEL_GOAL()"<<std::endl;
    
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

void p6_waypoint:: execute(const std::shared_ptr
        <rclcpp_action::ServerGoalHandle
            <software_training::action::Waypoint>> goal_handle) {


    const auto goal = goal_handle->get_goal();
    auto start_time = this->now();

    rclcpp::Rate rate{1};

    auto feedback = std::make_unique<software_training::action::Waypoint::Feedback>();
    auto result = std::make_unique<software_training::action::Waypoint::Result>();

    p6_waypoint::vec3d cur_lin{};
    p6_waypoint::vec3d cur_ang{}; 

    p6_waypoint::vec3d goal_lin{goal->linear_pos.x, goal->linear_pos.y, goal->linear_pos.z};
    p6_waypoint::vec3d goal_ang{goal->angular_pos.x, goal->angular_pos.y, goal->angular_pos.z};

    while (rclcpp::ok() && (cur_lin.or_less(goal_lin) || cur_ang.or_less(goal_ang))) {

        // cancel
        if (goal_handle->is_canceling()) {
            std::cout<<"GOAL_HANDLE->is_canceling()"<<std::endl;
            auto curr_time = this->now();
            auto time = curr_time - start_time;
            long int duration{time.nanoseconds()};
            result->duration = duration;
            goal_handle->canceled(std::move(result));
            return;
        }

        auto msg_cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
        vec3d temp_lin = cur_lin.advance(goal_lin);
        vec3d temp_ang = cur_ang.advance(goal_ang);

        // fill out message
        msg_cmd_vel->linear.x = temp_lin.x;
        msg_cmd_vel->linear.y = temp_lin.y;
        msg_cmd_vel->linear.z = temp_lin.z;

        msg_cmd_vel->angular.x = temp_ang.x;
        msg_cmd_vel->angular.y = temp_ang.y;
        msg_cmd_vel->angular.z = temp_ang.z;

        // send on /moving_turtle/cmd_vel
        this->publisher->publish(std::move(msg_cmd_vel));

        // update (static x + y get updated from subscriber)
        feedback->x_pos = this->p6_waypoint::x - cur_lin.x;
        feedback->y_pos = this->p6_waypoint::y - cur_lin.y;

        // some math for calculating angle?
        double magnitude = static_cast<double>(sqrt(pow(cur_lin.x, 2) + pow(cur_lin.y, 2) + pow(cur_lin.z, 2)));
        theta = acos(cur_lin.z / magnitude);

        feedback->theta_pos = this->p6_waypoint::theta - theta;

        // publish on feedback topic
        goal_handle->publish_feedback(std::move(feedback));
        rate.sleep();
    }

    // result
    if (rclcpp::ok()) {
        auto end = this->now();
        rclcpp::Duration duration = end - start_time;
        long int res_time{duration.nanoseconds()};
        result->duration = res_time;
        goal_handle->succeed(std::move(result));
        std::cout<<"goal done"<<std::endl;
    }
}

} // namespace composition

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(composition::p6_waypoint)