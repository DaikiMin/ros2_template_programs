#include <functional>
#include <memory>
#include <thread>

#include "ros2_custom_msg/action/timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "ros2_template_programs/component.h"

using Timer = ros2_custom_msg::action::Timer;
using GoalHandleTimer = rclcpp_action::ServerGoalHandle<Timer>;

namespace ros2_template_programs {
    class ActionServer : public rclcpp::Node {
        private:
            rclcpp_action::Server<Timer>::SharedPtr action_server_;
            double init_time;

            rclcpp_action::GoalResponse callbackGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Timer::Goal> goal);
            rclcpp_action::CancelResponse cancelGoal( const std::shared_ptr<GoalHandleTimer> goal_handle );
            void acceptedGoal( const std::shared_ptr<GoalHandleTimer> goal_handle );
            void execute( const std::shared_ptr<GoalHandleTimer> goal_handle );

        public:
            ROS2_TEMPLATE_PROGRAMS_PUBLIC
            explicit ActionServer( const rclcpp::NodeOptions & options = rclcpp::NodeOptions() );
    };
}

rclcpp_action::GoalResponse ros2_template_programs::ActionServer::callbackGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Timer::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %.2f", goal->target_time);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
rclcpp_action::CancelResponse ros2_template_programs::ActionServer::cancelGoal( const std::shared_ptr<GoalHandleTimer> goal_handle ) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}
void ros2_template_programs::ActionServer::acceptedGoal( const std::shared_ptr<GoalHandleTimer> goal_handle ) {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&ActionServer::execute, this, _1), goal_handle}.detach();
}
void ros2_template_programs::ActionServer::execute( const std::shared_ptr<GoalHandleTimer> goal_handle ) {
    RCLCPP_INFO(this->get_logger(), "Start Timer...");
    rclcpp::Rate loop_rate(10);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Timer::Feedback>();
    auto result = std::make_shared<Timer::Result>();

    double init_time = rclcpp::Clock().now().seconds();
    double curt_time = init_time;
    double target_time = goal->target_time;
    while ( (curt_time - init_time) < target_time ) {
        curt_time = rclcpp::Clock().now().seconds();
        // Check if there is a cancel request
        if (goal_handle->is_canceling()) {
            result->total_time = curt_time - init_time;
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
            return;
        }
        // Publish feedback
        feedback->elapsed_time = curt_time - init_time;
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "Publish the feedback : elapsed_time = %.2f", feedback->elapsed_time);

        loop_rate.sleep();
    }
    // Check if goal is done
    if (rclcpp::ok()) {
        result->total_time = curt_time - init_time;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "It is time to end : total_time = %.2f\n", result->total_time );
    }
}

ros2_template_programs::ActionServer::ActionServer(const rclcpp::NodeOptions & options ) : Node("action_server_template", options) {
    RCLCPP_INFO(this->get_logger(), "test");
    using namespace std::placeholders;
    action_server_ = rclcpp_action::create_server<Timer>(
        this,
        "timer_action_server",
        std::bind(&ActionServer::callbackGoal, this, _1, _2),
        std::bind(&ActionServer::cancelGoal, this, _1),
        std::bind(&ActionServer::acceptedGoal, this, _1));
}

RCLCPP_COMPONENTS_REGISTER_NODE(ros2_template_programs::ActionServer)