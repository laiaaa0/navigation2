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

namespace nav2_util
{

typedef std::vector<std::string> Tokens;
std::vector<std::string> Split(const std::string tokenstring, char delimiter);

void BringupLifecycleNodes(const std::vector<std::string> & node_names);
void BringupLifecycleNodes(const std::string & nodes)
{
  BringupLifecycleNodes(Split(nodes, ':'));
}

}  // namespace nav2_util

#endif  // NAV2_UTIL__LIFECYCLE_HPP_
