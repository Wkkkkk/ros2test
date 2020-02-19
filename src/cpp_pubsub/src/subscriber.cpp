//
// Created by zhihui on 1/14/20.
//

#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tutorials_msgs/msg/contact.hpp"

using std::placeholders::_1;

class ContactSubscriber : public rclcpp::Node {
public:
    ContactSubscriber()
            : Node("address_book_subscriber") {
        subscription_ = this->create_subscription<tutorials_msgs::msg::Contact>(
                "contact", 10, std::bind(&ContactSubscriber::topic_callback, this, _1));
    }

private:
    void topic_callback(const tutorials_msgs::msg::Contact::SharedPtr msg) const {
        std::cout << "I heard: " << msg->address << std::endl;
    }

    rclcpp::Subscription<tutorials_msgs::msg::Contact>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ContactSubscriber>());
    rclcpp::shutdown();
    return 0;
}