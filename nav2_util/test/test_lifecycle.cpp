// Copyright (c) 2019 Intel Corporation
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

#include "nav2_util/lifecycle.hpp"
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "gtest/gtest.h"

using nav2_util::BringupLifecycleNodes;
using nav2_util::Split;
using nav2_util::Tokens;

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

TEST(Split, SplitFunction)
{
  ASSERT_EQ(Split("", ':'), Tokens({""}));
  ASSERT_EQ(Split("foo", ':'), Tokens{"foo"});
  ASSERT_EQ(Split("foo:bar", ':'), Tokens({"foo", "bar"}));
  ASSERT_EQ(Split("foo:bar:", ':'), Tokens({"foo", "bar", ""}));
  ASSERT_EQ(Split(":", ':'), Tokens({"", ""}));
  ASSERT_EQ(Split("foo::bar", ':'), Tokens({"foo", "", "bar"}));
}

bool AreAllNodesActivated(std::vector<rclcpp_lifecycle::LifecycleNode::SharedPtr> & nodes) {
  for (const auto & node : nodes) {
    if (node->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      return false;
    }
  }
  return true;
}
void SpinNodesUntilActivated(std::vector<rclcpp_lifecycle::LifecycleNode::SharedPtr> nodes)
{
  rclcpp::executors::SingleThreadedExecutor exec;
  for (const auto & node : nodes) {
    exec.add_node(node->get_node_base_interface());
  }
  while(rclcpp::ok() && !AreAllNodesActivated(nodes)) {
    exec.spin_some();
  }
}

TEST(Lifecycle, interface)
{
  std::vector<rclcpp_lifecycle::LifecycleNode::SharedPtr> nodes;
  nodes.push_back(rclcpp_lifecycle::LifecycleNode::make_shared("foo"));
  nodes.push_back(rclcpp_lifecycle::LifecycleNode::make_shared("bar"));

  std::thread node_thread(SpinNodesUntilActivated, nodes);
  BringupLifecycleNodes("/foo:/bar");
  node_thread.join();
  SUCCEED();
}
