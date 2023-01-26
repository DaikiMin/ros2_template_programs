#include "ros2_template_programs/topic_subscriber_template.hpp"

void ros2_template_programs::TopicSubscriber::callbackMessage(const std_msgs::msg::String & msg) const {
    RCLCPP_INFO(this->get_logger(), "Message Subscribed: '%s'", msg.data.c_str());
}

ros2_template_programs::TopicSubscriber::TopicSubscriber() : Node("topic_subscriber_template") {
    sub_msg_ = this->create_subscription<std_msgs::msg::String>( "topic", 10, std::bind(&TopicSubscriber::callbackMessage, this, std::placeholders::_1 ));
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ros2_template_programs::TopicSubscriber>());
    rclcpp::shutdown();
    return 0;
}