// Copyright (c) 2018 Intel Corporation
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
#include <exception>
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node=rclcpp::Node::make_shared("minimal_subscriber");
    rclcpp::Rate loop_rate(1);
    while(rclcpp::ok()) {
      RCLCPP_INFO(node->get_logger(), "Node Time is %ld", node->now().nanoseconds());
      RCLCPP_INFO(node->get_logger(), "Clock Time is %ld", rclcpp::Clock(RCL_ROS_TIME).now().nanoseconds());
      rclcpp::spin_some(node);
      loop_rate.sleep();
    }
  } catch (...) {
    std::cerr << "caught exception" << std::endl;
  }
  rclcpp::shutdown();


  return 0;
}
