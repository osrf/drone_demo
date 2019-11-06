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

#ifndef DRONE_NODE__DRONE_NODE_HPP_
#define DRONE_NODE__DRONE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_gps_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>

#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <proposed_aerial_msgs/msg/flight_mode.hpp>
#include <proposed_aerial_msgs/msg/global_position.hpp>
#include <proposed_aerial_msgs/msg/attitude.hpp>
#include <proposed_aerial_msgs/srv/set_flight_mode.hpp>

#include <utils/geodetic_converter.hpp>
#include <tf2/transform_datatypes.h>
#include "tf2/utils.h"

using namespace std::chrono_literals;

class DroneNode : public rclcpp::Node
{
public:
  DroneNode();
  ~DroneNode();

private:

  // battery status attributes
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_pub_;
  rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr battery_status_sub_;

  // flight mode attributes
  rclcpp::Publisher<proposed_aerial_msgs::msg::FlightMode>::SharedPtr flight_mode_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr vehicle_land_detected_sub_;
  rclcpp::TimerBase::SharedPtr flight_mode_timer_;
  void flight_mode_timer_callback();
  int arming_state_;
  bool flying_;
  int nav_state_;

  // global position flight modes
  rclcpp::Subscription<proposed_aerial_msgs::msg::GlobalPosition>::SharedPtr global_position_sub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr vehicle_gps_sensor_pub_;
  int target_system_;
  std::shared_ptr<rclcpp::Service<proposed_aerial_msgs::srv::SetFlightMode>> flight_mode_service_;
  void set_fligh_mode_handle_service(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<proposed_aerial_msgs::srv::SetFlightMode::Request> request,
    const std::shared_ptr<proposed_aerial_msgs::srv::SetFlightMode::Response> response);

  // odometry
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;

  // goal
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_goal_sub_;
  std::shared_ptr<GeodeticConverter> geodetic_converter;
  rclcpp::Subscription<px4_msgs::msg::VehicleGpsPosition>::SharedPtr vehicle_gps_position_sub_;

  // attitude
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_atitude_sub_;
  rclcpp::Publisher<proposed_aerial_msgs::msg::Attitude>::SharedPtr vehicle_attitude_pub_;

};


#endif // DRONE_DEMO__DRONE_DEMO_HPP_
