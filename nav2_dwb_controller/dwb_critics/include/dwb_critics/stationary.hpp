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

#ifndef DWB_CRITICS__STATIONARY_HPP_
#define DWB_CRITICS__STATIONARY_HPP_

#include "dwb_core/trajectory_critic.hpp"

namespace dwb_critics
{
/**
 * @class StationaryCritic
 * @brief Penalize long periods of standing still
 */
class StationaryCritic : public dwb_core::TrajectoryCritic
{
public:
  void onInit() override;
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj) override;
  void debrief(const nav_2d_msgs::msg::Twist2D & cmd_vel) override;
protected:
  bool isTwistStationary(const nav_2d_msgs::msg::Twist2D & twist);
  double current_cost_;
  double epsilon_;
  double max_cost_;
};
}  // namespace dwb_critics

#endif  // DWB_CRITICS__STATIONARY_HPP_
