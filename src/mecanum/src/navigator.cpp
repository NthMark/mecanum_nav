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
using namespace std::chrono_literals;
using namespace std::placeholders;

enum class TaskResult
{
    UNKNOWN = 0,
    SUCCEEDED,
    CANCELED,
    FAILED
};
namespace navigator{
    class BasicNavigator : public rclcpp::Node
{
public:
    explicit BasicNavigator(): Node("basic_navigator")
    {   rclcpp::QoS amcl_pose_qos_(10);
        initial_pose_.header.frame_id = "map";
        amcl_pose_qos_
        .reliable()
        .transient_local()
        .keep_last(10);
        // amcl_pose_qos_.durability = rclcpp::DurabilityPolicy::TransientLocal;
        // amcl_pose_qos_.reliability = rclcpp::ReliabilityPolicy::Reliable;
        // amcl_pose_qos_.history = rclcpp::HistoryPolicy::KeepLast;
        // amcl_pose_qos_.depth = 1;

        initial_pose_received_ = false;
        nav_to_pose_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
        follow_waypoints_client_ = rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(this, "follow_waypoints");
        follow_path_client_ = rclcpp_action::create_client<nav2_msgs::action::FollowPath>(this, "follow_path");
        compute_path_to_pose_client_ = rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(this, "compute_path_to_pose");
        spin_client_ = rclcpp_action::create_client<nav2_msgs::action::Spin>(this, "spin");
        backup_client_ = rclcpp_action::create_client<nav2_msgs::action::BackUp>(this, "backup");

        // localization_pose_sub_ = this->create_subfscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        //     "amcl_pose", amcl_pose_qos_,
        //     std::bind(&BasicNavigator::_amclPoseCallback, this, _1));

        initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);

        change_maps_srv_ = this->create_client<nav2_msgs::srv::LoadMap>("/map_server/load_map");
        clear_costmap_global_srv_ = this->create_client<nav2_msgs::srv::ClearEntireCostmap>("/global_costmap/clear_entirely_global_costmap");
        clear_costmap_local_srv_ = this->create_client<nav2_msgs::srv::ClearEntireCostmap>("/local_costmap/clear_entirely_local_costmap");
        get_costmap_global_srv_ = this->create_client<nav2_msgs::srv::GetCostmap>("/global_costmap/get_costmap");
        get_costmap_local_srv_ = this->create_client<nav2_msgs::srv::GetCostmap>("/local_costmap/get_costmap");
    }

    // ~BasicNavigator()
    // {
    //     destroy_node();
    // }

    // void destroy_node()
    // {
    //     nav_to_pose_client_.reset();
    //     follow_waypoints_client_.reset();
    //     follow_path_client_.reset();
    //     compute_path_to_pose_client_.reset();
    //     spin_client_.reset();
    //     backup_client_.reset();
    //     rclcpp::shutdown();
        
    // }

    // void setInitialPose(const geometry_msgs::msg::PoseStamped &initial_pose)
    // {
    //     initial_pose_received_ = false;
    //     initial_pose_ = initial_pose;
    //     _setInitialPose();
    // }

    bool goToPose(const geometry_msgs::msg::PoseStamped &pose, const std::string &behavior_tree = "")
    {
        RCLCPP_DEBUG(this->get_logger(), "Waiting for 'NavigateToPose' action server");
        while (!nav_to_pose_client_->wait_for_action_server(1s))
        {
            RCLCPP_INFO(this->get_logger(), "'NavigateToPose' action server not available, waiting...");
        }

        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
        goal_msg.pose = pose;
        goal_msg.behavior_tree = behavior_tree;
        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback =
        std::bind(&BasicNavigator::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
        std::bind(&BasicNavigator::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
        std::bind(&BasicNavigator::result_callback, this, _1);
        RCLCPP_INFO(this->get_logger(), "Navigating to goal: %f %f...", pose.pose.position.x, pose.pose.position.y);
        auto send_goal_future = nav_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
        rclcpp::spin_until_future_complete(this->shared_from_this(), send_goal_future);
        return true;
    }

    // bool followWaypoints(const std::vector<geometry_msgs::msg::PoseStamped> &poses)
    // {
    //     RCLCPP_DEBUG(this->get_logger(), "Waiting for 'FollowWaypoints' action server");
    //     while (!follow_waypoints_client_->wait_for_action_server(1s))
    //     {
    //         RCLCPP_INFO(this->get_logger(), "'FollowWaypoints' action server not available, waiting...");
    //     }

    //     auto goal_msg = nav2_msgs::action::FollowWaypoints::Goal();
    //     goal_msg.poses = poses;

    //     RCLCPP_INFO(this->get_logger(), "Following %zu goals....", goal_msg.poses.size());
    //     auto send_goal_future = follow_waypoints_client_->async_send_goal(goal_msg, std::bind(&BasicNavigator::_feedbackCallback, this, _1));
    //     rclcpp::spin_until_future_complete(this->shared_from_this(), send_goal_future);
    //     goal_handle_ = send_goal_future.get();

    //     if (!goal_handle_)
    //     {
    //         RCLCPP_ERROR(this->get_logger(), "Following %zu waypoints request was rejected!", poses.size());
    //         return false;
    //     }

    //     result_future_ = goal_handle_->async_result();
    //     return true;
    // }

    // bool spin(float spin_dist = 1.57, int time_allowance = 10)
    // {
    //     RCLCPP_DEBUG(this->get_logger(), "Waiting for 'Spin' action server");
    //     while (!spin_client_->wait_for_action_server(1s))
    //     {
    //         RCLCPP_INFO(this->get_logger(), "'Spin' action server not available, waiting...");
    //     }

    //     auto goal_msg = nav2_msgs::action::Spin::Goal();
    //     goal_msg.target_yaw = spin_dist;
    //     goal_msg.time_allowance = rclcpp::Duration(time_allowance, 0);

    //     RCLCPP_INFO(this->get_logger(), "Spinning to angle %f....", goal_msg.target_yaw);
    //     auto send_goal_future = spin_client_->async_send_goal(goal_msg, std::bind(&BasicNavigator::_feedbackCallback, this, _1));
    //     rclcpp::spin_until_future_complete(this->shared_from_this(), send_goal_future);
    //     goal_handle_ = send_goal_future.get();

    //     if (!goal_handle_)
    //     {
    //         RCLCPP_ERROR(this->get_logger(), "Spin request was rejected!");
    //         return false;
    //     }

    //     result_future_ = goal_handle_->async_result();
    //     return true;
    // }

    // bool backup(float backup_dist = 0.15, float backup_speed = 0.025, int time_allowance = 10)
    // {
    //     RCLCPP_DEBUG(this->get_logger(), "Waiting for 'Backup' action server");
    //     while (!backup_client_->wait_for_action_server(1s))
    //     {
    //         RCLCPP_INFO(this->get_logger(), "'Backup' action server not available, waiting...");
    //     }

    //     auto goal_msg = nav2_msgs::action::BackUp::Goal();
    //     goal_msg.target.x = static_cast<double>(backup_dist);
    //     goal_msg.speed = static_cast<double>(backup_speed);
    //     goal_msg.time_allowance = rclcpp::Duration(time_allowance, 0);

    //     RCLCPP_INFO(this->get_logger(), "Backing up %f m at %f m/s....", goal_msg.target.x, goal_msg.speed);
    //     auto send_goal_future = backup_client_->async_send_goal(goal_msg, std::bind(&BasicNavigator::_feedbackCallback, this, _1));
    //     rclcpp::spin_until_future_complete(this->shared_from_this(), send_goal_future);
    //     goal_handle_ = send_goal_future.get();

    //     if (!goal_handle_)
    //     {
    //         RCLCPP_ERROR(this->get_logger(), "Backup request was rejected!");
    //         return false;
    //     }

    //     result_future_ = goal_handle_->async_result();
    //     return true;
    // }

    // bool followPath(const nav_msgs::msg::Path &path, const std::string &controller_id = "", const std::string &goal_checker_id = "")
    // {
    //     RCLCPP_DEBUG(this->get_logger(), "Waiting for 'FollowPath' action server");
    //     while (!follow_path_client_->wait_for_action_server(1s))
    //     {
    //         RCLCPP_INFO(this->get_logger(), "'FollowPath' action server not available, waiting...");
    //     }

    //     auto goal_msg = nav2_msgs::action::FollowPath::Goal();
    //     goal_msg.path = path;
    //     goal_msg.controller_id = controller_id;
    //     goal_msg.goal_checker_id = goal_checker_id;

    //     RCLCPP_INFO(this->get_logger(), "Following path....");
    //     auto send_goal_future = follow_path_client_->async_send_goal(goal_msg, std::bind(&BasicNavigator::_feedbackCallback, this, _1));
    //     rclcpp::spin_until_future_complete(this->shared_from_this(), send_goal_future);
    //     goal_handle_ = send_goal_future.get();

    //     if (!goal_handle_)
    //     {
    //         RCLCPP_ERROR(this->get_logger(), "FollowPath request was rejected!");
    //         return false;
    //     }

    //     result_future_ = goal_handle_->async_result();
    //     return true;
    // }

    // nav_msgs::msg::Path getPath(const geometry_msgs::msg::PoseStamped &start, const geometry_msgs::msg::PoseStamped &goal, const std::string &planner_id = "", const std::string &behavior_tree = "")
    // {
    //     RCLCPP_DEBUG(this->get_logger(), "Waiting for 'ComputePathToPose' action server");
    //     while (!compute_path_to_pose_client_->wait_for_action_server(1s))
    //     {
    //         RCLCPP_INFO(this->get_logger(), "'ComputePathToPose' action server not available, waiting...");
    //     }

    //     auto goal_msg = nav2_msgs::action::ComputePathToPose::Goal();
    //     goal_msg.start = start;
    //     goal_msg.goal = goal;
    //     goal_msg.planner_id = planner_id;
    //     goal_msg.behavior_tree = behavior_tree;

    //     RCLCPP_INFO(this->get_logger(), "Computing path to pose %f %f....", goal_msg.goal.pose.position.x, goal_msg.goal.pose.position.y);
    //     auto send_goal_future = compute_path_to_pose_client_->async_send_goal(goal_msg, std::bind(&BasicNavigator::_feedbackCallback, this, _1));
    //     rclcpp::spin_until_future_complete(this->shared_from_this(), send_goal_future);
    //     goal_handle_ = send_goal_future.get();

    //     if (!goal_handle_)
    //     {
    //         RCLCPP_ERROR(this->get_logger(), "ComputePathToPose request was rejected!");
    //         return nav_msgs::msg::Path();
    //     }

    //     result_future_ = goal_handle_->async_result();
    //     return goal_handle_->get_result().path;
    // }

    // TaskResult getResult(TaskResult result = TaskResult::UNKNOWN)
    // {
    //     rclcpp::spin_until_future_complete(this->shared_from_this(), result_future_);
    //     auto result_response = result_future_.get();

    //     switch (result_response.result->result)
    //     {
    //     case nav2_msgs::action::NavigateToPose::Result::SUCCEEDED:
    //         result = TaskResult::SUCCEEDED;
    //         break;
    //     case nav2_msgs::action::NavigateToPose::Result::CANCELED:
    //         result = TaskResult::CANCELED;
    //         break;
    //     default:
    //         result = TaskResult::FAILED;
    //         break;
    //     }
    //     return result;
    // }

    // bool waitUntilNav2Active()
    // {
    //     rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr get_state_srv =
    //         this->create_client<lifecycle_msgs::srv::GetState>("/lifecycle_manager_navigation/get_state");
    //     auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    //     auto response = std::make_shared<lifecycle_msgs::srv::GetState::Response>();

    //     while (!get_state_srv->wait_for_service(1s))
    //     {
    //         if (!rclcpp::ok())
    //         {
    //             RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
    //             return false;
    //         }
    //         RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    //     }

    //     auto result_future = get_state_srv->async_send_request(request);
    //     rclcpp::spin_until_future_complete(this->shared_from_this(), result_future);
    //     response = result_future.get();

    //     while (response->current_state.label != "active")
    //     {
    //         RCLCPP_INFO(this->get_logger(), "Waiting for Nav2 to become active...");
    //         std::this_thread::sleep_for(1s);
    //         result_future = get_state_srv->async_send_request(request);
    //         rclcpp::spin_until_future_complete(this->shared_from_this(), result_future);
    //         response = result_future.get();
    //     }

    //     RCLCPP_INFO(this->get_logger(), "Nav2 is active!");
    //     return true;
    // }
    // void _amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    // {
    //     if (!initial_pose_received_)
    //     {
    //         initial_pose_ = *msg;
    //         initial_pose_received_ = true;
    //     }
    // }

    // void _setInitialPose()
    // {
    //     initial_pose_pub_->publish(initial_pose_);
    //     std::this_thread::sleep_for(1s);
    // }

    // void _feedbackCallback(std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::Feedback::ConstSharedPtr> feedback)
    // {
    //     RCLCPP_INFO(this->get_logger(), "Received feedback...");
    // }
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
  void goal_response_callback(GoalHandlePose::SharedPtr goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandlePose::SharedPtr,
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(), "Received feedback...");
  }

  void result_callback(const GoalHandlePose::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    rclcpp::shutdown();
  }
  
  
};
}
RCLCPP_COMPONENTS_REGISTER_NODE(navigator::BasicNavigator)
// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);

//     auto navigator = std::make_shared<BasicNavigator>();

//     if (!navigator->waitUntilNav2Active())
//     {
//         return 1;
//     }

//     geometry_msgs::msg::PoseStamped goal_pose;
//     goal_pose.header.frame_id = "map";
//     goal_pose.pose.position.x = 2.0;
//     goal_pose.pose.position.y = 3.0;
//     goal_pose.pose.orientation.w = 1.0;

//     navigator->goToPose(goal_pose);

//     TaskResult result = navigator->getResult();
//     if (result == TaskResult::SUCCEEDED)
//     {
//         RCLCPP_INFO(navigator->get_logger(), "Navigation succeeded!");
//     }
//     else
//     {
//         RCLCPP_INFO(navigator->get_logger(), "Navigation failed.");
//     }

//     rclcpp::shutdown();
//     return 0;
// }
