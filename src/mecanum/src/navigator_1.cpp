// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#include <memory>
#include <memory>
#include <string>
#include <vector>
#include "rclcpp_components/register_node_macro.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "builtin_interfaces/msg/duration.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/spin.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"
#include "nav2_msgs/srv/load_map.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "mecanum/visibility_control.h"

namespace action_tutorials_cpp
{
class FibonacciActionClient : public rclcpp::Node
{
private:
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_to_pose_client_;
    rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr follow_waypoints_client_;
    rclcpp_action::Client<nav2_msgs::action::FollowPath>::SharedPtr follow_path_client_;
    rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SharedPtr compute_path_to_pose_client_;
    rclcpp_action::Client<nav2_msgs::action::Spin>::SharedPtr spin_client_;
    rclcpp_action::Client<nav2_msgs::action::BackUp>::SharedPtr backup_client_;

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr localization_pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;

    rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr change_maps_srv_;
    rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_costmap_global_srv_;
    rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_costmap_local_srv_;
    rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr get_costmap_global_srv_;
    rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr get_costmap_local_srv_;
    geometry_msgs::msg::PoseStamped initial_pose_;
    bool initial_pose_received_;

    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle_;
    std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::Result::SharedPtr> result_future_;
    using GoalHandlePose = rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;

  NAVIGATOR_PUBLIC
  explicit FibonacciActionClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : Node("fibonacci_action_client", node_options)
  {
        // amcl_pose_qos_.durability = rclcpp::DurabilityPolicy::TransientLocal;
        // amcl_pose_qos_.reliability = rclcpp::ReliabilityPolicy::Reliable;
        // amcl_pose_qos_.history = rclcpp::HistoryPolicy::KeepLast;
        // amcl_pose_qos_.depth = 1
        this->nav_to_pose_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
  }

  
};  // class FibonacciActionClient

}  // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(action_tutorials_cpp::FibonacciActionClient)
