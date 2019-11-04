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

#include <sensor_msgs/msg/battery_state.hpp>

#include <proposed_aerial_msgs/msg/flight_mode.hpp>
#include <proposed_aerial_msgs/msg/global_position.hpp>
#include <proposed_aerial_msgs/srv/set_flight_mode.hpp>

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
  int target_system_;
  std::shared_ptr<rclcpp::Service<proposed_aerial_msgs::srv::SetFlightMode>> flight_mode_service_;
  void set_fligh_mode_handle_service(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<proposed_aerial_msgs::srv::SetFlightMode::Request> request,
    const std::shared_ptr<proposed_aerial_msgs::srv::SetFlightMode::Response> response);


};


#endif // DRONE_DEMO__DRONE_DEMO_HPP_
