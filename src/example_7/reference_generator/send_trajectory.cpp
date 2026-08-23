// Copyright 2023 ros2_control Development Team
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

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <chrono>
#include <vector>

namespace
{
trajectory_msgs::msg::JointTrajectoryPoint make_point(
  const std::vector<double> & positions,
  const std::vector<double> & velocities,
  const int sec,
  const uint32_t nanosec = 0)
{
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = positions;
  point.velocities = velocities;
  point.time_from_start.sec = sec;
  point.time_from_start.nanosec = nanosec;
  return point;
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("send_trajectory");
  auto pub = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/joint_trajectory_controller/joint_trajectory", 10);

  trajectory_msgs::msg::JointTrajectory trajectory_msg;
  trajectory_msg.header.stamp = node->now();
  trajectory_msg.joint_names = {"joint_1", "joint_2", "joint_3", "joint_4"};

  // Gentle 3-DoF profile: hold -> move slowly -> return slowly.
  trajectory_msg.points.push_back(make_point({0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}, 0));
  trajectory_msg.points.push_back(make_point({0.2, -0.2, -0.4, 0.8}, {0.0, 0.0, 0.0, 0.0}, 3));
  trajectory_msg.points.push_back(make_point({0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}, 6));

  auto started = node->now();
  while (pub->get_subscription_count() == 0)
  {
    if (node->now() - started > rclcpp::Duration(10, 0))
    {
      RCLCPP_ERROR(
        node->get_logger(), "No subscribers connected after waiting for 10 seconds. Exiting.");
      return 1;
    }
    RCLCPP_INFO(
      node->get_logger(), "Waiting for subscribers to connect to topic %s", pub->get_topic_name());
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }

  RCLCPP_INFO(
    node->get_logger(), "Publishing trajectory with length %ld", trajectory_msg.points.size());
  
  pub->publish(trajectory_msg);

  rclcpp::sleep_for(std::chrono::milliseconds(500));
  rclcpp::shutdown();

  return 0;
}
