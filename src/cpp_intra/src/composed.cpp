// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "cpp_intra/publisher_node.hpp"
#include "cpp_intra/subscriber_node.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exec;

    // magic!
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);

    auto publisher_node = std::make_shared<PublisherNode>(options);
    auto subscriber_node = std::make_shared<SubscriberNode>(options);
    exec.add_node(publisher_node);
    exec.add_node(subscriber_node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
