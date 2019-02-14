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

#include "nav2_util/node_utils.hpp"
#include <chrono>
#include <string>

using std::chrono::high_resolution_clock;
using std::to_string;
using std::string;

namespace nav2_util
{

std::string SanitizeNodeName(const std::string & potential_node_name)
{
  std::string node_name(potential_node_name);
  std::replace_if(begin(node_name), end(node_name),
    [](auto c) {return !isalnum(c);},
    '_');  // we replace a non alphanumeric with _
  return node_name;
}

std::string TimeToString(size_t len)
{
  string output(len, '0');  // prefill the string with zeros
  auto timepoint = high_resolution_clock::now();
  auto timecount = timepoint.time_since_epoch().count();
  auto timestring = to_string(timecount);
  if (timestring.length() >= len) {
    output.replace(0, len,
      timestring,
      timestring.length() - len, len);
  } else {
    output.replace(len - timestring.length(), timestring.length(),
      timestring,
      0, timestring.length());
  }
  return output;
}

std::string GenerateInternalNodeName(const std::string & prefix)
{
  return SanitizeNodeName(prefix) + "_" + TimeToString(8);
}

rclcpp::Node::SharedPtr GenerateInternalNode(const std::string & prefix)
{
  rclcpp::NodeOptions options;
  options.use_global_arguments(false);
  options.start_parameter_services(false);
  options.start_parameter_event_publisher(false);
  return rclcpp::Node::make_shared(GenerateInternalNodeName(prefix));
}

}  // namespace nav2_util
