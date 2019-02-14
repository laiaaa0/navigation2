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
#include "lifecycle_msgs/srv/change_state.hpp"
#include "nav2_util/service_client.hpp"

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
  ServiceClient<lifecycle_msgs::srv::ChangeState> sc(node_name + "/change_state");
  sc.waitForService();
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;
  sc.invoke(request);
  request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
  sc.invoke(request);
}

void BringupLifecycleNodes(const std::vector<std::string> & node_names)
{
  for (const auto & node_name : node_names) {
    BringupLifecycleNode(node_name);
  }
}

}  // namespace nav2_util
