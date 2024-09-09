#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "lifecycle_msgs/msg/transition.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "rcutils/logging_macros.h"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;
using namespace std::placeholders;
class LifecycleControl : public rclcpp_lifecycle::LifecycleNode
{
public:
using GoalHandlePose = rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
  explicit LifecycleControl(const std::string & node_name, bool intra_process_comms = false)
  : rclcpp_lifecycle::LifecycleNode(node_name,
      rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
  {
    a_=0.13/2;
    d_=0.3;
    l_=0.25;
    // target_frame_ = this->declare_parameter<std::string>("target_frame", "base_link");
    // tf_buffer_ =
    //   std::make_unique<tf2_ros::Buffer>(this->get_clock());
    // tf_listener_ =
    //   std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    // rclcpp::QoS amcl_pose_qos_(10);
    //     amcl_pose_qos_
    //     .reliable()
    //     .transient_local()
    //     .keep_last(10);

    // nav_to_pose_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
  }
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
  
  // bool goToPose(const geometry_msgs::msg::PoseStamped &pose, const std::string &behavior_tree = "")
  //   {
  //       RCLCPP_DEBUG(this->get_logger(), "Waiting for 'NavigateToPose' action server");
  //       while (!nav_to_pose_client_->wait_for_action_server(1s))
  //       {
  //           RCLCPP_INFO(this->get_logger(), "'NavigateToPose' action server not available, waiting...");
  //       }

  //       auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
  //       goal_msg.pose = pose;
  //       goal_msg.behavior_tree = behavior_tree;
  //       auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  //       send_goal_options.goal_response_callback =
  //       std::bind(&LifecycleControl::goal_response_callback, this, _1);
  //       send_goal_options.feedback_callback =
  //       std::bind(&LifecycleControl::feedback_callback, this, _1, _2);
  //       send_goal_options.result_callback =
  //       std::bind(&LifecycleControl::result_callback, this, _1);
  //       RCLCPP_INFO(this->get_logger(), "Navigating to goal: %f %f...", pose.pose.position.x, pose.pose.position.y);
  //       auto send_goal_future = nav_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
  //       rclcpp::spin_until_future_complete(this->shared_from_this(), send_goal_future);
  //       goal_handle_ = send_goal_future.get();
  //       if (!goal_handle_)
  //       {
  //           RCLCPP_ERROR(this->get_logger(), "Goal to %f %f was rejected!", pose.pose.position.x, pose.pose.position.y);
  //           return false;
  //       }

  //       return true;
  //   }
  // void move_map(double x,double y,double theta){
  //   goal_pose_.header.stamp = this->get_clock()->now();
  //   goal_pose_.header.frame_id = "odom";
  //   // initial_pose_.child_frame_id = "base_footprint";
  //   goal_pose_.pose.position.x = x;
  //   goal_pose_.pose.position.y = y;  
  //   tf2::Quaternion q;
  //   q.setRPY(0,0,theta);
  //   goal_pose_.pose.orientation.x = q.x();
  //   goal_pose_.pose.orientation.y = q.y();
  //   goal_pose_.pose.orientation.z = q.z();
  //   goal_pose_.pose.orientation.w = q.w();
  //   goToPose(goal_pose_);
  // }
  void
  publish()
  {
    static int count=0;
    auto msg = std::make_unique<geometry_msgs::msg::Twist>();
    // std::string fromFrameRel = target_frame_.c_str();
    // std::string toFrameRel = "origin";
    // geometry_msgs::msg::TransformStamped t;
    // t = tf_buffer_->lookupTransform(
    //         toFrameRel, fromFrameRel,
    //         tf2::TimePointZero);
            
    // RCLCPP_INFO(get_logger(),"linear x: %.2f, linear y: %.2f, angular z: %.2f",t.transform.translation.x,t.transform.translation.y,t.transform.rotation.z);
    if (count<5){
        //Forward
        msg->linear.x=0.64;
        msg->linear.y=0;
        msg->angular.z=0;
    }
    else if (count<10){
        //Backward
        msg->linear.x=-0.64;
        msg->linear.y=0;
        msg->angular.z=0;
    }
    else if (count<15){
        //Right
        msg->linear.x=0;
        msg->linear.y=-0.64;
        msg->angular.z=0;
    }
    else if (count<20){
        //Forward right
        msg->linear.x=0.32;
        msg->linear.y=-0.32;
        msg->angular.z=0;
    }
    else if (count <25){
        //Forward-left
        msg->linear.x=0.32;
        msg->linear.y=0.32;
        msg->angular.z=0;
    }
    else if (count<30){
        //Turning-right
        msg->linear.x=0;
        msg->linear.y=0;
        msg->angular.z=-0.8;
    }
    else if (count<35){
        //Curved trajectory
        msg->linear.x=0.48;
        msg->linear.y=0;
        msg->angular.z=-0.2;
    }
    // msg->linear.x=-0.32;
    // msg->linear.y=0.32;
    // msg->angular.z=-0.8;
    double_t w1,w2,w3,w4;
    // Print the current state for demo purposes
    if (!pub_->is_activated()) {
      RCLCPP_INFO(
        get_logger(), "Lifecycle publisher is currently inactive. Messages are not published.");
    } else {
        w1= 1/a_*(msg->linear.x+msg->linear.y+(-d_+l_)*msg->angular.z);
        w2= 1/a_*(msg->linear.x-msg->linear.y+(-d_+l_)*msg->angular.z);
        w3= 1/a_*(msg->linear.x+msg->linear.y+(d_-l_)*msg->angular.z);
        w4= 1/a_*(msg->linear.x-msg->linear.y+(d_-l_)*msg->angular.z);

        // RCLCPP_INFO(get_logger(),"Angular velocities of 4 wheels respectively are w1 = %2.f \t w2 = %2.f \t w3 = %2.f \t w4 = %2.f",w1,w2,w3,w4);
        // RCLCPP_INFO(
        // get_logger(), "Lifecycle publisher is active. Publishing: linear: x= %f y= %f z= %f \n -----\n angular: x= %f y= %f z=%f", \
        msg->linear.x,msg->linear.y,msg->linear.z,msg->angular.x,msg->angular.y,msg->angular.z);
    }
    count+=1;
    pub_->publish(std::move(msg));
  }
  void pos_callback(nav_msgs::msg::Odometry::ConstSharedPtr msg){
    tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w
    );
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    RCLCPP_INFO(get_logger(),"position x: %.2f  y: %.2f \n orientation roll: %.2f pitch: %.2f yaw: %.2f ",msg->pose.pose.position.x,msg->pose.pose.position.y,\
    roll,pitch,yaw);
  }
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &)
  {
    // move_map(3,0,0);
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    sub_=this-> create_subscription<nav_msgs::msg::Odometry>(
      "odom",10,std::bind(&LifecycleControl::pos_callback,this,std::placeholders::_1)
    );
    timer_ = this->create_wall_timer(
      1s, std::bind(&LifecycleControl::publish, this));
    timer_->cancel();
    RCLCPP_INFO(get_logger(), "on_configure() is called.");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state)
  {
    timer_->reset();
    LifecycleNode::on_activate(state);

    RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");

    std::this_thread::sleep_for(2s);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state)
  {
    auto msg = std::make_unique<geometry_msgs::msg::Twist>();
    msg->linear.x=0;
    msg->linear.y=0;
    msg->angular.z=0;
    pub_->publish(std::move(msg));
    LifecycleNode::on_deactivate(state);

    RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &)
  {
    auto msg = std::make_unique<geometry_msgs::msg::Twist>();
    msg->linear.x=0;
    msg->linear.y=0;
    msg->angular.z=0;
    pub_->publish(std::move(msg));
    timer_.reset();
    pub_.reset();
    RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called.");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & state)
  {
    auto msg = std::make_unique<geometry_msgs::msg::Twist>();
    msg->linear.x=0;
    msg->linear.y=0;
    msg->angular.z=0;
    pub_->publish(std::move(msg));
    timer_.reset();
    pub_.reset();
    RCUTILS_LOG_INFO_NAMED(
      get_name(),
      "on shutdown is called from state %s.",
      state.label().c_str());

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

private:
    // std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    // std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string target_frame_;
    std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> sub_;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>> pub_;
    // rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_to_pose_client_;
    // rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle_;
    // // std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::Result::SharedPtr> result_future_;
    // std::shared_future<LifecycleControl::GoalHandlePose::WrappedResult> result_future_;
    geometry_msgs::msg::PoseStamped goal_pose_;
    std::shared_ptr<rclcpp::TimerBase> timer_;
    double_t a_;
    double_t d_;
    double_t l_;
};
int main(int argc, char * argv[])
{
  // force flush of the stdout buffer.
  // this ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exe;

  std::shared_ptr<LifecycleControl> lc_node =
    std::make_shared<LifecycleControl>("lc_Control");

  exe.add_node(lc_node->get_node_base_interface());

  exe.spin();

  rclcpp::shutdown();

  return 0;
}
