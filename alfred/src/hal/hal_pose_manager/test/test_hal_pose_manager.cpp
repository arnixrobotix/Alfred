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

#include "hal_pose_manager_tests.hpp"

namespace hal
{
namespace pose_manager
{
namespace test
{

using namespace std::placeholders;

PoseManagerCheckerNode::PoseManagerCheckerNode()
: rclcpp::Node{"hal_pose_manager_checker_node"},
  changeStateClient{this->create_client<lifecycle_msgs::srv::ChangeState>(
      "hal_pose_manager_node/change_state")},
  wheelsVelocityCmdSubscriber{this->create_subscription<HalMotorControlCommandMsg_t>(
      "wheelsVelocityCmd", 10,
      std::bind(&PoseManagerCheckerNode::wheelsVelocityCmdReader, this, _1))},
  odometrySubscriber{this->create_subscription<OdometryMsg_t>(
      "odometry", 10, std::bind(&PoseManagerCheckerNode::odometryReader, this, _1) )},
  wheelsVelocityCommand{0.0, 0.0},
  odometry{{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}, {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}}}
{
}

void PoseManagerCheckerNode::changePoseManagerNodeToState(std::uint8_t transition)
{
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition;
  auto result = changeStateClient->async_send_request(request);
}

void PoseManagerCheckerNode::wheelsVelocityCmdReader(const HalMotorControlCommandMsg_t & msg)
{
  wheelsVelocityCommand.left = msg.motor_left_command;
  wheelsVelocityCommand.right = msg.motor_right_command;
}

void PoseManagerCheckerNode::odometryReader(const OdometryMsg_t & msg)
{
  auto extractTwistValues = [](auto twistRead) {
      Twist twist{{twistRead.linear.x, twistRead.linear.y, twistRead.linear.z},
        {twistRead.angular.x, twistRead.angular.y, twistRead.angular.z}};
      return twist;
    };

  auto extractPoseValues = [](auto poseRead) {
      Pose pose{{poseRead.position.x, poseRead.position.y, poseRead.position.z},
        {poseRead.orientation.x, poseRead.orientation.y, poseRead.orientation.z,
          poseRead.orientation.w}};
      return pose;
    };

  odometry.twist = extractTwistValues(msg.twist.twist);
  odometry.pose = extractPoseValues(msg.pose.pose);
}

// TEST_F(PoseManagerActivatedTest, WheelsVelocityCmdPublished)
// {
//   TwistMsg_t twistTestMessage;
//   twistTestMessage.twist.linear.x = TWIST_COMMAND_1_M_PER_S;
//   twistTestMessage.twist.angular.x = TWIST_COMMAND_2_M_PER_S;
//   poseManager->computeAndPublishwheelsVelocityCmd(twistTestMessage);
//   executorPoseManager.spin_some();

//   ASSERT_DOUBLE_EQ(poseManagerChecker->wheelsVelocityCommand.right, TWIST_COMMAND_1_M_PER_S);
//   ASSERT_DOUBLE_EQ(poseManagerChecker->wheelsVelocityCommand.left, TWIST_COMMAND_1_M_PER_S);
// }

TEST_F(PoseManagerActivatedTest, PositiveXPositionOdometryPublished)
{
  HalMotorControlEncodersMsg_t encoderCountFirstMessage;
  encoderCountFirstMessage.header.stamp.nanosec = 10 * MS_TO_NS;
  encoderCountFirstMessage.motor_left_encoder_count = 2080;
  encoderCountFirstMessage.motor_right_encoder_count = 2080;

  poseManager->publishOdometry(encoderCountFirstMessage);
  executorPoseManager.spin_some();

  ASSERT_NEAR(poseManagerChecker->odometry.pose.position.x, 0.01, 1e-3);
  ASSERT_NEAR(poseManagerChecker->odometry.pose.position.y, 0.0, 1e-3);
}

TEST_F(PoseManagerActivatedTest, PositiveXPositionOdometryPublishedTwoMessages)
{
  HalMotorControlEncodersMsg_t encoderCountFirstMessage;
  encoderCountFirstMessage.header.stamp.nanosec = 10 * MS_TO_NS;
  encoderCountFirstMessage.motor_left_encoder_count = 2080;
  encoderCountFirstMessage.motor_right_encoder_count = 2080;

  HalMotorControlEncodersMsg_t encoderCountSecondMessage;
  encoderCountSecondMessage.header.stamp.nanosec = 20 * MS_TO_NS;
  encoderCountSecondMessage.motor_left_encoder_count = 6240;
  encoderCountSecondMessage.motor_right_encoder_count = 6240;

  poseManager->publishOdometry(encoderCountFirstMessage);
  executorPoseManager.spin_some();

  ASSERT_NEAR(poseManagerChecker->odometry.pose.position.x, 0.01, 1e-3);
  ASSERT_NEAR(poseManagerChecker->odometry.pose.position.y, 0.0, 1e-3);

  poseManager->publishOdometry(encoderCountSecondMessage);
  executorPoseManager.spin_some();

  ASSERT_NEAR(poseManagerChecker->odometry.pose.position.x, 0.03, 1e-3);
  ASSERT_NEAR(poseManagerChecker->odometry.pose.position.y, 0.0, 1e-3);
}

TEST_F(PoseManagerActivatedTest, NegativeXPositionOdometryPublished)
{
  HalMotorControlEncodersMsg_t encoderCountFirstMessage;
  encoderCountFirstMessage.header.stamp.nanosec = 10 * MS_TO_NS;
  encoderCountFirstMessage.motor_left_encoder_count = -2080;
  encoderCountFirstMessage.motor_right_encoder_count = -2080;

  poseManager->publishOdometry(encoderCountFirstMessage);
  executorPoseManager.spin_some();

  ASSERT_NEAR(poseManagerChecker->odometry.pose.position.x, -0.01, 1e-3);
  ASSERT_NEAR(poseManagerChecker->odometry.pose.position.y, 0.0, 1e-3);
}

TEST_F(PoseManagerActivatedTest, XYPositionNoMovementOdometryPublished)
{
  HalMotorControlEncodersMsg_t encoderCountFirstMessage;
  encoderCountFirstMessage.header.stamp.nanosec = 10 * MS_TO_NS;
  encoderCountFirstMessage.motor_left_encoder_count = 2080;
  encoderCountFirstMessage.motor_right_encoder_count = -2080;

  poseManager->publishOdometry(encoderCountFirstMessage);
  executorPoseManager.spin_some();

  ASSERT_NEAR(poseManagerChecker->odometry.pose.position.x, 0.0, 1e-3);
  ASSERT_NEAR(poseManagerChecker->odometry.pose.position.y, 0.0, 1e-3);
}

TEST_F(PoseManagerActivatedTest, XYPositionOdometryPublished)
{
  for (int i = 1; i <= 80; i++) {
    HalMotorControlEncodersMsg_t encoderCountMessage;
    encoderCountMessage.header.stamp.nanosec = i * MS_TO_NS;
    encoderCountMessage.motor_left_encoder_count = 0;
    encoderCountMessage.motor_right_encoder_count = i * 612;

    poseManager->publishOdometry(encoderCountMessage);
    executorPoseManager.spin_some();
  }

  // Robot turned 90° to the left around its left wheel
  ASSERT_NEAR(poseManagerChecker->odometry.pose.position.x, robotWidth_m / 2.0, 1e-3);
  ASSERT_NEAR(poseManagerChecker->odometry.pose.position.y, robotWidth_m / 2.0, 1e-3);
}

}  // namespace test
}  // namespace pose_manager
}  // namespace hal

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  auto result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return result;
}
