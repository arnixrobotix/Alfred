// Copyright (c) 2023 Arnix Robotix
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

#ifndef HAL_POSE_MANAGER_TESTS_HPP_
#define HAL_POSE_MANAGER_TESTS_HPP_

#include <memory>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"

#include "hal_pose_manager.hpp"

namespace hal
{
namespace pose_manager
{
namespace test
{

#define TWIST_COMMAND_1_M_PER_S 1.0
#define TWIST_COMMAND_2_M_PER_S 2.0
#define MS_TO_NS 1000000

class PoseManagerCheckerNode : public rclcpp::Node
{
public:
  PoseManagerCheckerNode();
  ~PoseManagerCheckerNode() = default;

  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr changeStateClient;
  rclcpp::Subscription<OdometryMsg_t>::SharedPtr odometrySubscriber;

  void changePoseManagerNodeToState(std::uint8_t transition);

  void odometryReader(const OdometryMsg_t & msg);

  Odometry odometry;
};

/* Test fixture */
class PoseManagerActivatedTest : public testing::Test
{
protected:
  std::shared_ptr<HalPoseManager> poseManager;
  std::shared_ptr<PoseManagerCheckerNode> poseManagerChecker;
  rclcpp::executors::SingleThreadedExecutor executorPoseManager;

  void SetUp()
  {
    poseManagerChecker = std::make_shared<PoseManagerCheckerNode>();
    poseManager = std::make_shared<HalPoseManager>();

    executorPoseManager.add_node(poseManager->get_node_base_interface());
    executorPoseManager.add_node(poseManagerChecker);

    poseManagerChecker->changePoseManagerNodeToState(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    executorPoseManager.spin_some();
    poseManagerChecker->changePoseManagerNodeToState(
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    executorPoseManager.spin_some();
  }

  void TearDown()
  {
    poseManagerChecker->changePoseManagerNodeToState(
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN);
    executorPoseManager.spin_some();
    executorPoseManager.cancel();
    executorPoseManager.remove_node(poseManager->get_node_base_interface());
    executorPoseManager.remove_node(poseManagerChecker);
    poseManager.reset();
    poseManagerChecker.reset();
  }
};

}  // namespace test
}  // namespace pose_manager
}  // namespace hal

#endif  // HAL_POSE_MANAGER_TESTS_HPP_
