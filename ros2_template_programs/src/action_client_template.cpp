#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "ros2_custom_msg/action/timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using Timer = ros2_custom_msg::action::Timer;
using GoalHandleTimer = rclcpp_action::ClientGoalHandle<Timer>;

namespace ros2_template_programs {
    class ActionClient : public rclcpp::Node {
        private:
            rclcpp_action::Client<Timer>::SharedPtr client_ptr_;
            rclcpp::TimerBase::SharedPtr timer_;

            void callbackGoalResponse(const GoalHandleTimer::SharedPtr & goal_handle);
            void callbackFeedback( GoalHandleTimer::SharedPtr, const std::shared_ptr<const Timer::Feedback> feedback);
            void callbackResult(const GoalHandleTimer::WrappedResult & result);

        public:
            explicit ActionClient( const rclcpp::NodeOptions & options = rclcpp::NodeOptions() );
            void sendGoal();
    };
}

void ros2_template_programs::ActionClient::callbackGoalResponse(const GoalHandleTimer::SharedPtr & goal_handle) {
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
}

void ros2_template_programs::ActionClient::callbackFeedback( GoalHandleTimer::SharedPtr, const std::shared_ptr<const Timer::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(), "Received feedback : elapsed_time = %.2f", feedback->elapsed_time);
}

void ros2_template_programs::ActionClient::callbackResult(const GoalHandleTimer::WrappedResult & result) {
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
    }
    RCLCPP_INFO(this->get_logger(), "Received result : Total_time = %.2f\n", result.result->total_time );
    rclcpp::shutdown();
}

ros2_template_programs::ActionClient::ActionClient( const rclcpp::NodeOptions & options ) : Node( "action_client_template", options) {
    RCLCPP_INFO(this->get_logger(), "test");
    client_ptr_ = rclcpp_action::create_client<Timer>( this, "timer_action_server" );
    timer_ = create_wall_timer( std::chrono::milliseconds(500), std::bind(&ActionClient::sendGoal, this));
}

void ros2_template_programs::ActionClient::sendGoal() {
    using namespace std::placeholders;

    timer_->cancel();

    if (!client_ptr_->wait_for_action_server()) {
        RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
        rclcpp::shutdown();
    }

    auto goal_msg = Timer::Goal();
    declare_parameter( "target_time", 5.0);
    goal_msg.target_time = get_parameter("target_time").as_double();

    RCLCPP_INFO(get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Timer>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&ActionClient::callbackGoalResponse, this, _1);
    send_goal_options.feedback_callback = std::bind(&ActionClient::callbackFeedback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&ActionClient::callbackResult, this, _1);
    client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

RCLCPP_COMPONENTS_REGISTER_NODE(ros2_template_programs::ActionClient)