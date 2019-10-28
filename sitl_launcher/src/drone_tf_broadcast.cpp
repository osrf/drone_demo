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
#include <chrono>
using namespace std::chrono_literals;

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("drone_odom_broadcast"),
   clock_(get_clock()),
   buffer(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME))
  {

    rclcpp::QoS  odom_and_imu_qos(rclcpp::KeepLast(50));

    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(buffer);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(std::string("odom"), odom_and_imu_qos);

    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    declare_parameter("odom_frame", "odom", descriptor);
    declare_parameter("vehicle_odometry", "/vehicle_odometry", descriptor);
    declare_parameter("base_link_frame", "plane_1_base_link", descriptor);

    odom_frame_ = "odom";
    get_parameter("odom_frame", odom_frame_);
    base_link_frame_ = "plane_1_base_link";
    get_parameter("base_link_frame", base_link_frame_);

    std::string vehicle_odometry_str = "/vehicle_odometry";
    get_parameter("vehicle_odometry", vehicle_odometry_str);

    subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
      vehicle_odometry_str, 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));

  }
  void init_tf_broadcaster()
   {
     tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());
   }

private:
  void topic_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) const
  {

    auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();
    odom_msg->header.frame_id = odom_frame_;
    odom_msg->child_frame_id = base_link_frame_;
    auto odom_tf_msg = std::make_shared<geometry_msgs::msg::TransformStamped>();
    odom_tf_msg->header.frame_id = odom_frame_;
    odom_tf_msg->child_frame_id = base_link_frame_;

    tf2::Quaternion q_orig, q_new;

    // ENU ---> NED
    // roll ---->roll
    // pitch ----->-pitch
    // yaw ----->-yaw+1.57.

    q_orig = tf2::Quaternion(msg->q[1], msg->q[2], msg->q[3], msg->q[0]);
    tf2::Matrix3x3 m(q_orig);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw, 2);
    q_new.setRPY(roll, -pitch, -yaw+1.57);

    // Stuff and publish /odom
    odom_msg->header.stamp = clock_->now();
    odom_msg->pose.pose.position.x = msg->y;
    odom_msg->pose.pose.position.y = msg->x;
    odom_msg->pose.pose.position.z = -msg->z;
    odom_msg->pose.pose.orientation.x = q_new.x();
    odom_msg->pose.pose.orientation.y = q_new.y();
    odom_msg->pose.pose.orientation.z = q_new.z();
    odom_msg->pose.pose.orientation.w = q_new.w();

    for (unsigned int i = 0; i < odom_msg->pose.covariance.size(); ++i) {
      odom_msg->pose.covariance[i] = msg->pose_covariance[i];
    }

    // Stuff and publish /tf
    odom_tf_msg->header.stamp = odom_msg->header.stamp;
    odom_tf_msg->transform.translation.x = msg->y;
    odom_tf_msg->transform.translation.y = msg->x;
    odom_tf_msg->transform.translation.z = -msg->z;
    odom_tf_msg->transform.rotation.x = q_new.x();
    odom_tf_msg->transform.rotation.y = q_new.y();
    odom_tf_msg->transform.rotation.z = q_new.z();
    odom_tf_msg->transform.rotation.w = q_new.w();

    odom_pub_->publish(std::move(odom_msg));
    tf_broadcaster_->sendTransform(*odom_tf_msg);

  }
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  tf2_ros::Buffer buffer;
  rclcpp::Clock::SharedPtr clock_;

  std::string odom_frame_;
  std::string gyro_link_frame_;
  std::string base_link_frame_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalSubscriber>();
  node->init_tf_broadcaster();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
