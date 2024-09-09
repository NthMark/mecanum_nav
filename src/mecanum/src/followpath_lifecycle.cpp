#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include "lifecycle_msgs/msg/transition.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "rcutils/logging_macros.h"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
using namespace std::chrono_literals;

class LifecycleFollowPath : public rclcpp_lifecycle::LifecycleNode
{
public:
    explicit LifecycleFollowPath(const std::string &node_name, bool intra_process_comms = false)
        : rclcpp_lifecycle::LifecycleNode(node_name,
                                          rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
    {
        a_ = 0.13 / 2;
        d_ = 0.3;
        l_ = 0.25;
        std::vector<double> temp{0, 0, 0};
        eta.emplace_back(std::move(temp));
        
        temp.clear();
        lambda.emplace_back(1);
        lambda.emplace_back(1);
        lambda.emplace_back(8);
    }

    void
    publish()
    {
        static double timestep = 0;
        auto msg = std::make_unique<geometry_msgs::msg::Twist>();
        msg->linear.x = 0.5;
        double w1, w2, w3, w4;
        // Print the current state for demo purposes
        if (!pub_->is_activated())
        {
            RCLCPP_INFO(
                get_logger(), "Lifecycle publisher is currently inactive. Messages are not published.");
        }
        else
        {
            if (timestep > 20)
            {
                msg->linear.x = 0;
                msg->linear.y = 0;
                msg->angular.z = 0;
            }
            else
            {
                std::vector<double> temp_ele;
                double psi = eta.at(eta.size()-1).at(2);
                // eta_d.emplace_back(10 * sin(0.5 * timestep));
                // eta_d.emplace_back(10 - 10 * cos(0.5 * timestep));
                // eta_d.emplace_back(0);
                // eta_dot_d.emplace_back(5 * cos(0.5 * timestep));
                // eta_dot_d.emplace_back(5 * sin(0.5 * timestep));
                // eta_dot_d.emplace_back(0);
                eta_d.emplace_back(timestep);
                eta_d.emplace_back(0);
                eta_d.emplace_back(0);
                eta_dot_d.emplace_back(1);
                eta_dot_d.emplace_back(0);
                eta_dot_d.emplace_back(0);

                temp_ele.emplace_back(cos(psi));
                temp_ele.emplace_back(-sin(psi));
                temp_ele.emplace_back(0);
                jacobian.emplace_back(std::move(temp_ele));
                temp_ele.clear();
                temp_ele.emplace_back(sin(psi));
                temp_ele.emplace_back(cos(psi));
                temp_ele.emplace_back(0);
                jacobian.emplace_back(std::move(temp_ele));
                temp_ele.clear();
                temp_ele.emplace_back(0);
                temp_ele.emplace_back(0);
                temp_ele.emplace_back(1);
                jacobian.emplace_back(std::move(temp_ele));
                temp_ele.clear();

                // inverse jacobi
                temp_ele.emplace_back(cos(psi) / (cos(psi) * cos(psi) + sin(psi) * sin(psi)));
                temp_ele.emplace_back(sin(psi) / (cos(psi) * cos(psi) + sin(psi) * sin(psi)));
                temp_ele.emplace_back(0);
                inv_jacobian.emplace_back(std::move(temp_ele));
                temp_ele.clear();
                temp_ele.emplace_back(-sin(psi) / (cos(psi) * cos(psi) + sin(psi) * sin(psi)));
                temp_ele.emplace_back(cos(psi) / (cos(psi) * cos(psi) + sin(psi) * sin(psi)));
                temp_ele.emplace_back(0);
                inv_jacobian.emplace_back(std::move(temp_ele));
                temp_ele.clear();
                temp_ele.emplace_back(0);
                temp_ele.emplace_back(0);
                temp_ele.emplace_back(1);
                inv_jacobian.emplace_back(std::move(temp_ele));
                temp_ele.clear();
                eta_tildax = eta_d.at(0) - eta.at(eta.size()-1).at(0);
                eta_tilday = eta_d.at(1) - eta.at(eta.size()-1).at(1);
                eta_tildarotz = eta_d.at(2) - eta.at(eta.size()-1).at(2);

                msg->linear.x =inv_jacobian.at(0).at(0)*(eta_dot_d.at(0)+lambda.at(0)*eta_tildax)+
                inv_jacobian.at(0).at(1)*(eta_dot_d.at(1))+
                inv_jacobian.at(0).at(2)*(eta_dot_d.at(2));

                msg->linear.y =inv_jacobian.at(1).at(0)*(eta_dot_d.at(0))+
                inv_jacobian.at(1).at(1)*(eta_dot_d.at(1)+lambda.at(1)*eta_tilday)+
                inv_jacobian.at(1).at(2)*(eta_dot_d.at(2));

                msg->angular.z =inv_jacobian.at(2).at(0)*(eta_dot_d.at(0))+
                inv_jacobian.at(2).at(1)*(eta_dot_d.at(1))+
                inv_jacobian.at(2).at(2)*(eta_dot_d.at(2)+lambda.at(2)*eta_tildarotz);

                w1 = 1 / a_ * (msg->linear.x + msg->linear.y + (-d_ + l_) * msg->angular.z);
                w2 = 1 / a_ * (msg->linear.x - msg->linear.y + (-d_ + l_) * msg->angular.z);
                w3 = 1 / a_ * (msg->linear.x + msg->linear.y + (d_ - l_) * msg->angular.z);
                w4 = 1 / a_ * (msg->linear.x - msg->linear.y + (d_ - l_) * msg->angular.z);
            }

            RCLCPP_INFO(get_logger(), "Angular velocities of 4 wheels respectively are w1 = %2.f \t w2 = %2.f \t w3 = %2.f \t w4 = %2.f", w1, w2, w3, w4);
            RCLCPP_INFO(
                get_logger(), "Lifecycle publisher is active. Publishing: linear: x= %f y= %f z= %f \n -----\n angular: x= %f y= %f z=%f",
                msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x, msg->angular.y, msg->angular.z);
            timestep += 1;
            eta_d.clear();
            eta_dot_d.clear();
            jacobian.clear();
            inv_jacobian.clear();
        }

        pub_->publish(std::move(msg));
    }
    void pos_callback(nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        RCLCPP_INFO(get_logger(), "position x: %.2f  y: %.2f \n orientation roll: %.2f pitch: %.2f yaw: %.2f ", msg->pose.pose.position.x, msg->pose.pose.position.y,
                    roll, pitch, yaw);
        std::vector<double> temp;
        temp.emplace_back(msg->pose.pose.position.x);
        temp.emplace_back(msg->pose.pose.position.y);
        temp.emplace_back(yaw);
        eta.emplace_back(std::move(temp));
        if (eta.size()>10){
            RCLCPP_INFO(get_logger(),"greater than 10\n");
            eta.erase(eta.begin());
        }
        temp.clear();
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "%.2f %.2f %.2f",eta.at(0).at(0),eta.at(0).at(1),eta.at(0).at(2) );
        rclcpp::QoS qos(rclcpp::KeepLast{7});
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", qos);
        sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", rclcpp::SensorDataQoS(), std::bind(&LifecycleFollowPath::pos_callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(
            1s, std::bind(&LifecycleFollowPath::publish, this));
        // timer_->cancel();
        RCLCPP_INFO(get_logger(), "on_configure() is called.");

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &state)
    {
        LifecycleNode::on_activate(state);

        RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");

        std::this_thread::sleep_for(2s);
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &state)
    {
        auto msg = std::make_unique<geometry_msgs::msg::Twist>();
        msg->linear.y = 0;
        pub_->publish(std::move(msg));
        LifecycleNode::on_deactivate(state);

        RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State &)
    {
        auto msg = std::make_unique<geometry_msgs::msg::Twist>();
        msg->linear.y = 0;
        pub_->publish(std::move(msg));
        timer_.reset();
        pub_.reset();
        RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called.");

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State &state)
    {
        auto msg = std::make_unique<geometry_msgs::msg::Twist>();
        msg->linear.y = 0;
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
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>> pub_;
    std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> sub_;
    std::shared_ptr<rclcpp::TimerBase> timer_;
    double a_;
    double d_;
    double l_;
    double gamma1, gamma2, gamma3;
    double eta_tildax, eta_tilday, eta_tildarotz;
    double real_x, real_y, real_yaw;
    std::vector<double> eta_d;
    std::vector<double> eta_dot_d;
    std::vector<double> lambda;
    std::vector<std::vector<double>> jacobian;
    std::vector<std::vector<double>> inv_jacobian;
    std::vector<std::vector<double>> eta;
};
int main(int argc, char *argv[])
{
    // force flush of the stdout buffer.
    // this ensures a correct sync of all prints
    // even when executed simultaneously within the launch file.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;

    std::shared_ptr<LifecycleFollowPath> lc_node =
        std::make_shared<LifecycleFollowPath>("lc_FollowPath");

    exe.add_node(lc_node->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();

    return 0;
}
