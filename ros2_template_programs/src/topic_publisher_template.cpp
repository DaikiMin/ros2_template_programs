#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace ros2_template_programs {
    class TopicPublisher  : public rclcpp::Node {
        private:
            rclcpp::TimerBase::SharedPtr timer_;
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_msg_ ;
            std_msgs::msg::String msg_;
            void callbackTimer();

        public:
            TopicPublisher ();
    };
}

void ros2_template_programs::TopicPublisher::callbackTimer() {
    RCLCPP_INFO(this->get_logger(), "Message Published: '%s'", msg_.data.c_str());
    pub_msg_ ->publish(msg_);
}

ros2_template_programs::TopicPublisher::TopicPublisher() : Node("topic_publisher_template") {
    pub_msg_  = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&TopicPublisher::callbackTimer, this));
    declare_parameter( "message", "Hello, World!");
    msg_.data = get_parameter("message").as_string();
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ros2_template_programs::TopicPublisher>());
    rclcpp::shutdown();
    return 0;
}