//
// Created by zhihui on 1/14/20.
//

#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tutorials_msgs/msg/contact.hpp"

using namespace std::chrono_literals;

class ContactPublisher : public rclcpp::Node {
public:
    ContactPublisher()
            : Node("address_book_publisher") {
        publisher_ = this->create_publisher<tutorials_msgs::msg::Contact>("contact", 10);

        timer_ = this->create_wall_timer(1s, std::bind(&ContactPublisher::timer_callback, this));
    }

private:
    void timer_callback() {
        auto msg = tutorials_msgs::msg::Contact();

        msg.first_name = "John";
        msg.last_name = "Doe";
        msg.age = 30;
        msg.gender = msg.MALE;
        msg.address = "street " + std::to_string(count_++);

        std::cout << "Publishing Contact First:" << msg.first_name << "  Last:" << msg.last_name << std::endl;

        publisher_->publish(msg);
    }

    rclcpp::Publisher<tutorials_msgs::msg::Contact>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_ = 0;
};


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto publisher_node = std::make_shared<ContactPublisher>();

    rclcpp::spin(publisher_node);

    return 0;
}