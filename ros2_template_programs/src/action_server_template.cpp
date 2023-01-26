#include <functional>
#include <memory>
#include <thread>

#include "ros2_custom_msg/action/timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "ros2_template_programs/visibility_control.h"

using Timer = ros2_custom_msg::action::Timer;
using GoalHandleTimer = rclcpp_action::ServerGoalHandle<Timer>;


namespace ros_template_programs {
    class ActionServer : public rclcpp::Node {
        private:
            rclcpp_action::Server<Timer>::SharedPtr action_server_;
            double init_time;

            void callbackGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Timer::Goal> goal);
            void cancelGoal( const std::shared_ptr<GoalHandleTimer> goal_handle );
            void acceptedGoal( const std::shared_ptr<GoalHandleTimer> goal_handle );
            void execute( const std::shared_ptr<GoalHandleTimer> goal_handle );

        public:
            ROS2_TEMPLATE_PROGRAMS_PUBLIC
            explicit ActionServer( const rclcpp::NodeOptions & options = rclcpp::NodeOptions() );
    };
}

ros_template_programs::ActionServer::ActionServer(const rclcpp::NodeOptions & options ) : Node("action_server_template", options) {
    using std::placeholders::_1;
    using std::placeholders::_2;
    // action_server_ = rclcpp_action::create_server<Timer>(
    //     this,
    //     "timer_action_server",
    //     std::bind(&ActionServer::callbackGoal, this, _1, _2),
    //     std::bind(&ActionServer::cancelGoal, this, _1),
    //     std::bind(&ActionServer::acceptedGoal, this, _1));
}

RCLCPP_COMPONENTS_REGISTER_NODE(ros_template_programs::ActionServer)