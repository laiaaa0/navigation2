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
#include <vector>
#include <string>
#include <thread>
#include <chrono>
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "nav2_util/service_client.hpp"
#include "nav2_util/node_utils.hpp"

using nav2_util::GenerateInternalNode;

namespace nav2_util
{

Tokens Split(const std::string tokenstring, char delimiter)
{
  Tokens tokens;

  size_t current_pos = 0;
  size_t pos = 0;
  while ((pos = tokenstring.find(delimiter, current_pos)) != std::string::npos) {
    tokens.push_back(tokenstring.substr(current_pos, pos - current_pos));
    current_pos = pos + 1;
  }
  tokens.push_back(tokenstring.substr(current_pos));
  return tokens;
}

void BringupLifecycleNode(const std::string & node_name)
{
  LifecycleServiceClient sc(node_name);
  sc.ChangeState(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  sc.ChangeState(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  while (sc.GetState() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void BringupLifecycleNodes(const std::vector<std::string> & node_names)
{
  for (const auto & node_name : node_names) {
    BringupLifecycleNode(node_name);
  }
}

LifecycleServiceClient::LifecycleServiceClient(const std::string & node_name)
: node_(GenerateInternalNode(node_name + "_lifecycle_client")),
  change_state_(node_name + "/change_state", node_),
  get_state_(node_name + "/get_state", node_)
{
}

void LifecycleServiceClient::ChangeState(
  const uint8_t newState,
  const std::chrono::seconds timeout)
{
  change_state_.waitForService(timeout);
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = newState;
  change_state_.invoke(request, timeout);
}

uint8_t LifecycleServiceClient::GetState(
  const std::chrono::seconds timeout)
{
  get_state_.waitForService(timeout);
  auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
  auto result = get_state_.invoke(request, timeout);
  return result->current_state.id;
}

}  // namespace nav2_util
