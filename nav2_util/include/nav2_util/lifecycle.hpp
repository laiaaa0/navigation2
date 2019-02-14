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

#ifndef NAV2_UTIL__LIFECYCLE_HPP_
#define NAV2_UTIL__LIFECYCLE_HPP_

#include <vector>
#include <string>
#include <chrono>
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "nav2_util/service_client.hpp"

namespace nav2_util
{

typedef std::vector<std::string> Tokens;
std::vector<std::string> Split(const std::string tokenstring, char delimiter);

void BringupLifecycleNodes(const std::vector<std::string> & node_names);
void BringupLifecycleNodes(const std::string & nodes)
{
  BringupLifecycleNodes(Split(nodes, ':'));
}

class LifecycleServiceClient
{
public:
  LifecycleServiceClient(const std::string & node_name);

  void ChangeState(
    const uint8_t newState,  // takes a lifecycle_msgs::msg::Transition id
    const std::chrono::seconds timeout = std::chrono::seconds::max());

  // returns a lifecycle_msgs::msg::State id
  uint8_t GetState(const std::chrono::seconds timeout = std::chrono::seconds::max());

protected:
  rclcpp::Node::SharedPtr node_;
  ServiceClient<lifecycle_msgs::srv::ChangeState> change_state_;
  ServiceClient<lifecycle_msgs::srv::GetState> get_state_;
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__LIFECYCLE_HPP_
