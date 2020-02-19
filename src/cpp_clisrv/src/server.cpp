//
// Created by zhihui on 1/14/20.
//

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tutorials_msgs/srv/add_two_floats.hpp"

using AddTwoFloats = tutorials_msgs::srv::AddTwoFloats;
rclcpp::Node::SharedPtr g_node = nullptr;

void handle_service(
        const std::shared_ptr <rmw_request_id_t> request_header,
        const std::shared_ptr <AddTwoFloats::Request> request,
        const std::shared_ptr <AddTwoFloats::Response> response) {
    (void) request_header;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "request: a: %ld, b: %ld", request->a, request->b);
    response->sum = request->a + request->b;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr <rclcpp::Node> node = rclcpp::Node::make_shared("minimal_service");
    auto service = node->create_service<AddTwoFloats>("add_two_floats", handle_service);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
