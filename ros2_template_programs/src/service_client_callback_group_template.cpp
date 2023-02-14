#include "ros2_template_programs/service_client_callback_group_template.hpp"

void ros2_template_programs::ServiceClient::callbackTimer() {
    if ( count_%4 == 0 ) {
        request_->expression.calculate = "+";
    } else if ( count_%4 == 1 ) {
        request_->expression.calculate = "-";
    } else if ( count_%4 == 2 ) {
        request_->expression.calculate = "*";
    } else {
        request_->expression.calculate = "/";
    }
    auto future = client_->async_send_request(request_);
    std::future_status status = future.wait_for(10s);
    if (status == std::future_status::ready) {
        RCLCPP_INFO(this->get_logger(), "Call the calculate_two_numbers server\nExpression :%.2f %s %.2f\nResult : %.2f",
    request_->expression.a, request_->expression.calculate.c_str(), request_->expression.b, future.get()->result );
    } else {
        RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
    count_++;
}

ros2_template_programs::ServiceClient::ServiceClient() : Node("service_client_callback_group_template") {
    client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    client_ = this->create_client<ros2_custom_msg::srv::Calculation>("calculate_two_numbers", rmw_qos_profile_services_default, client_cb_group_ );
    while (!client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        else RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }
    timer_ = this->create_wall_timer(1s, std::bind(&ServiceClient::callbackTimer, this), timer_cb_group_);
    declare_parameter( "a", 1.0);
    declare_parameter( "b", 1.0);
    request_ = std::make_shared<ros2_custom_msg::srv::Calculation::Request>();
    request_->expression.a = get_parameter("a").as_double();
    request_->expression.b = get_parameter("b").as_double();
    count_ = 0;
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto client_node = std::make_shared<ros2_template_programs::ServiceClient>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(client_node);
    RCLCPP_INFO(client_node->get_logger(), "Starting client node, shut down with CTRL-C");
    executor.spin();
    RCLCPP_INFO(client_node->get_logger(), "Keyboard interrupt, shutting down.\n");
    rclcpp::shutdown();
    return 0;
}