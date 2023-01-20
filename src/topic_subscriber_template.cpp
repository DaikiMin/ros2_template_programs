#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class TopicSubscriber : public rclcpp::Node {
    private:
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_msg_;
        void callbackMessage(const std_msgs::msg::String & msg) const;
    public:
        TopicSubscriber();
};

void TopicSubscriber::callbackMessage(const std_msgs::msg::String & msg) const {
    RCLCPP_INFO(this->get_logger(), "Message Subscribed: '%s'", msg.data.c_str());
}

TopicSubscriber::TopicSubscriber() : Node("topic_subscriber_template") {
    sub_msg_ = this->create_subscription<std_msgs::msg::String>( "topic", 10, std::bind(&TopicSubscriber::callbackMessage, this, _1));
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TopicSubscriber>());
    rclcpp::shutdown();
    return 0;
}