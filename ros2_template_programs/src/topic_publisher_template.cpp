#include "ros2_template_programs/topic_publisher_template.hpp"

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