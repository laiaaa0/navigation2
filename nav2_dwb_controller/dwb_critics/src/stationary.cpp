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

#include "dwb_critics/stationary.hpp"
#include "dwb_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace dwb_critics
{
void StationaryCritic::onInit()
{
  // Scale is set to 0 by default, so if it was not set otherwise, set to 0
  nh_->get_parameter_or(name_ + ".scale", scale_, 0.0);

  nh_->get_parameter_or(name_ + ".zero_velocity_tolerance", epsilon_, 1e-4);
  nh_->get_parameter_or(name_ + ".max_stationary_cost", max_cost_, 20.0);
  current_cost_ = 0;
}

double StationaryCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj)
{
  if (isTwistStationary(traj.velocity)) {
    if (current_cost_ > max_cost_) {
      throw nav_core2::IllegalTrajectoryException(name_, "Excessive stationary time.");
    }
    return 0;
  } else {
    return 0;
  }
}

void StationaryCritic::debrief(const nav_2d_msgs::msg::Twist2D & cmd_vel)
{
  if (isTwistStationary(cmd_vel)) {
    ++current_cost_;
  } else {
    current_cost_ = 0; // if we've moved again, reset the penalty.
  }
}

bool StationaryCritic::isTwistStationary(const nav_2d_msgs::msg::Twist2D & twist)
{
  if((fabs(twist.x) < epsilon_) && (fabs(twist.y) < epsilon_) && (fabs(twist.theta) < epsilon_)) {
    return true;
  } else {
    return false;
  }
}

}  // namespace dwb_critics

PLUGINLIB_EXPORT_CLASS(dwb_critics::StationaryCritic, dwb_core::TrajectoryCritic)
