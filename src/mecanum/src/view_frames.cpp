#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

class TFFrameLister : public rclcpp::Node
{
public:
  TFFrameLister()
  : Node("tf_frame_lister"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&TFFrameLister::list_frames, this));
  }

private:
  void list_frames()
  {
    std::vector<std::string> frames = tf_buffer_.getAllFrameNames();

    RCLCPP_INFO(this->get_logger(), "Listing all frames:");
    for (const auto& frame : frames)
    {
      RCLCPP_INFO(this->get_logger(), "Frame: %s", frame.c_str());
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TFFrameLister>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
