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

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber"),
   clock(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)),
   buffer(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME))
  {
    // clock = ;
    subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
      "/vehicle_odometry", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(buffer);

  }
  void init_tf_broadcaster()
   {
     tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());
   }

private:
  void topic_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) const
  {

    // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = clock->now();
    t.header.frame_id = "world";
    t.child_frame_id = "base_link";
    t.transform.translation.x = msg->x;
    t.transform.translation.y = msg->y;
    t.transform.translation.z = -msg->z;
    t.transform.rotation.x = msg->q[1];
    t.transform.rotation.y = msg->q[2];
    t.transform.rotation.z = msg->q[3];
    t.transform.rotation.w = msg->q[0];
    tf_broadcaster_->sendTransform(t);

    // geometry_msgs::msg::TransformStamped tfGeom;
    // try {
        //
        // builtin_interfaces::msg::Time time_stamp = clock->now();
        // tf2::TimePoint time_point = tf2::TimePoint(
        //     std::chrono::seconds(time_stamp.sec) +
        //     std::chrono::nanoseconds(time_stamp.nanosec));
        // tfGeom = buffer.lookupTransform(
        //     "rotor_0", "base_link", time_point, tf2::Duration(100ms));
        // tf_broadcaster_->sendTransform(tfGeom);
        //
        // printf("%.5f\t%.5f\t%.5f\t%.5f\t%.5f\t%.5f\t%.5f\n",
        //        tfGeom.transform.translation.x,
        //        tfGeom.transform.translation.y,
        //        tfGeom.transform.translation.z,
        //        t.transform.rotation.x,
        //        t.transform.rotation.y,
        //        t.transform.rotation.z,
        //        t.transform.rotation.w);

    // } catch (tf2::TransformException &e) {
    //   printf("ERROR!\n");
    // }

    // //rotor0
    // t.header.stamp = clock->now();
    // t.header.frame_id = "base_link";
    // t.child_frame_id = "rotor_0";
    // t.transform.translation.x = 0.13;
    // t.transform.translation.y = -0.22;
    // t.transform.translation.z = 0.023;
    // t.transform.rotation.x = 0;
    // t.transform.rotation.y = 0;
    // t.transform.rotation.z = 0;
    // t.transform.rotation.w = 1;
    // tf_broadcaster_->sendTransform(t);
    //
    // //rotor1
    // t.header.stamp = clock->now();
    // t.header.frame_id = "base_link";
    // t.child_frame_id = "rotor_1";
    // t.transform.translation.x = -0.13;
    // t.transform.translation.y = 0.22;
    // t.transform.translation.z = 0.023;
    // t.transform.rotation.x = 0;
    // t.transform.rotation.y = 0;
    // t.transform.rotation.z = 0;
    // t.transform.rotation.w = 1;
    // tf_broadcaster_->sendTransform(t);
    //
    // //rotor1
    // t.header.stamp = clock->now();
    // t.header.frame_id = "base_link";
    // t.child_frame_id = "rotor_2";
    // t.transform.translation.x = 0.13;
    // t.transform.translation.y = 0.22;
    // t.transform.translation.z = 0.023;
    // t.transform.rotation.x = 0;
    // t.transform.rotation.y = 0;
    // t.transform.rotation.z = 0;
    // t.transform.rotation.w = 1;
    // tf_broadcaster_->sendTransform(t);
    //
    // //rotor1
    // t.header.stamp = clock->now();
    // t.header.frame_id = "base_link";
    // t.child_frame_id = "rotor_3";
    // t.transform.translation.x = -0.13;
    // t.transform.translation.y = -0.22;
    // t.transform.translation.z = 0.023;
    // t.transform.rotation.x = 0;
    // t.transform.rotation.y = 0;
    // t.transform.rotation.z = 0;
    // t.transform.rotation.w = 1;
    // tf_broadcaster_->sendTransform(t);
  }
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscription_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  tf2_ros::Buffer buffer;
  rclcpp::Clock::SharedPtr clock;
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
