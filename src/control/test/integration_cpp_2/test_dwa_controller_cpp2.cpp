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

#include <gtest/gtest.h>

#include <cstdlib>
#include <experimental/filesystem>
#include <rclcpp/rclcpp.hpp>
#include <control/FollowPathTaskClient.hpp>
#include <iostream>

using namespace std;
namespace fs = std::experimental::filesystem;

class PlannerNodeLauncher
{
public:
  PlannerNodeLauncher()
  {
    auto launchFile = fs::path(getenv("TEST_LAUNCH_DIR")) / fs::path("dwa_controller_node.launch.py");
    ros_launcher_pid = subprocess("ros2 launch " + launchFile.string());
  }

  ~PlannerNodeLauncher()
  {
    kill(-ros_launcher_pid, SIGINT);
  }
protected:
  pid_t subprocess(std::string command)
  {
    auto pid = fork();
    if(pid == 0) // Child
    {
      setpgid(getpid(), getpid());
      system(command.c_str());
      exit(0);
    }
    return pid;
  }
  pid_t ros_launcher_pid;
};

//By makinging it a global variable, this has executable scope rather than being
//restarted every test.
PlannerNodeLauncher g_launcher;

class TestNode : public ::testing::Test
{
public:
  TestNode() {
    rclcpp::init(0, nullptr);
    node = rclcpp::Node::make_shared("dwa_controller_test");
    client = std::make_unique<FollowPathTaskClient>("DwaController", node.get());
    while(node->count_subscribers("/DwaController_command") < 1)
    {
      rclcpp::spin_some(node);
    }
  }
  ~TestNode() {
    rclcpp::shutdown();
  }

protected:
  std::shared_ptr<rclcpp::Node> node;
  std::unique_ptr<FollowPathTaskClient> client;
};

TEST_F(TestNode, ResultReturned)
{
  FollowPathCommand c;
  client->executeAsync(std::make_shared<FollowPathCommand>(c));
  FollowPathResult r;
  while(client->waitForResult(std::make_shared<FollowPathResult>(r), 1) == TaskStatus::RUNNING)
  {
    rclcpp::spin_some(node);
  }
  SUCCEED();
}
