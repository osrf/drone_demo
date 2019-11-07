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

#include "drone_node/drone_node.hpp"

DroneNode::DroneNode():
  Node("drone_node")
{
  declare_parameter("target_system");
  get_parameter("target_system", target_system_);

  printf("%s target_system %d\n", get_namespace(), target_system_);

  battery_state_pub_ = create_publisher<sensor_msgs::msg::BatteryState>("battery_state", 10);

  battery_status_sub_ = create_subscription<px4_msgs::msg::BatteryStatus>(
            "battery_status",
          10,
          [this](px4_msgs::msg::BatteryStatus::ConstSharedPtr msg) {
            sensor_msgs::msg::BatteryState msg_to_send;
            msg_to_send.header.stamp = get_clock()->now();
            msg_to_send.voltage = msg->voltage_v;
            msg_to_send.temperature = msg->temperature;
            msg_to_send.capacity = msg->capacity;
            msg_to_send.percentage = msg->remaining*100;

            for(unsigned int i = 0; i < msg->voltage_cell_v.size(); i++){
              msg_to_send.cell_voltage.push_back(msg->voltage_cell_v[i]);
            }

            msg_to_send.serial_number = std::to_string(msg->serial_number);

            battery_state_pub_->publish(std::move(msg_to_send));
        });

  flight_mode_pub_ = create_publisher<proposed_aerial_msgs::msg::FlightMode>("flight_mode", 10);

  flight_mode_timer_ = this->create_wall_timer(
    100ms, std::bind(&DroneNode::flight_mode_timer_callback, this));

  vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
            "vehicle_status",
            10,
            [this](px4_msgs::msg::VehicleStatus::ConstSharedPtr msg) {
              arming_state_ = msg->arming_state;
              nav_state_ = msg->nav_state;
      });
  vehicle_land_detected_sub_ = create_subscription<px4_msgs::msg::VehicleLandDetected>(
            "vehicle_land_detected",
            10,
            [this](px4_msgs::msg::VehicleLandDetected::ConstSharedPtr msg) {
              flying_ = !msg->landed;
      });

  odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>("odometry", 10);
  vehicle_odometry_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
            "vehicle_odometry",
            10,
            [this](px4_msgs::msg::VehicleOdometry::ConstSharedPtr msg) {
              auto msg_to_send = std::make_unique<nav_msgs::msg::Odometry>();
              msg_to_send->header.stamp = get_clock()->now();
              msg_to_send->pose.pose.position.x = msg->x;
              msg_to_send->pose.pose.position.y = msg->y;
              msg_to_send->pose.pose.position.z = msg->z;
              msg_to_send->pose.pose.orientation.x = msg->q[0];
              msg_to_send->pose.pose.orientation.y = msg->q[1];
              msg_to_send->pose.pose.orientation.z = msg->q[2];
              msg_to_send->pose.pose.orientation.w = msg->q[3];
              for(unsigned int i = 0; i < msg->pose_covariance.size(); i++){
                msg_to_send->pose.covariance[i] = msg->pose_covariance[i];
              }
              msg_to_send->twist.twist.linear.x = msg->vx;
              msg_to_send->twist.twist.linear.y = msg->vy;
              msg_to_send->twist.twist.linear.z = msg->vz;
              msg_to_send->twist.twist.angular.x = msg->rollspeed;
              msg_to_send->twist.twist.angular.y = msg->pitchspeed;
              msg_to_send->twist.twist.angular.z = msg->yawspeed;
              for(unsigned int i = 0; i < msg->velocity_covariance.size(); i++){
                msg_to_send->twist.covariance[i] = msg->velocity_covariance[i];
              }
              odometry_pub_->publish(std::move(msg_to_send));
      });

  vehicle_command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>("vehicle_command", 10);
  global_position_sub_ = create_subscription<proposed_aerial_msgs::msg::GlobalPosition>(
            "command_gps_pose",
            10,
            [this](proposed_aerial_msgs::msg::GlobalPosition::ConstSharedPtr msg) {
              px4_msgs::msg::VehicleCommand msg_vehicle_command;

              if(!flying_){
                //ARM
                msg_vehicle_command.timestamp = get_clock()->now().nanoseconds()/1000;
                msg_vehicle_command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
                msg_vehicle_command.param1 = 1;
                msg_vehicle_command.confirmation = 1;
                msg_vehicle_command.source_system = 255;
                msg_vehicle_command.target_system = target_system_;
                msg_vehicle_command.target_component = 1;
                msg_vehicle_command.from_external = true;
                vehicle_command_pub_->publish(msg_vehicle_command);

                // take off
                msg_vehicle_command.timestamp = get_clock()->now().nanoseconds()/1000;
                msg_vehicle_command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF;
                msg_vehicle_command.param1 = 0.1;
                msg_vehicle_command.param2 = 0;
                msg_vehicle_command.param3 = 0;
                msg_vehicle_command.param4 = msg->yaw;
                msg_vehicle_command.param5 = msg->latitude;
                msg_vehicle_command.param6 = msg->longitude;
                msg_vehicle_command.param7 = 3.0;
                msg_vehicle_command.confirmation = 1;
                msg_vehicle_command.source_system = 255;
                msg_vehicle_command.target_system = target_system_;
                msg_vehicle_command.target_component = 1;
                msg_vehicle_command.from_external = true;
                vehicle_command_pub_->publish(msg_vehicle_command);
              }

              msg_vehicle_command.timestamp = get_clock()->now().nanoseconds()/1000;
              msg_vehicle_command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_REPOSITION;
              msg_vehicle_command.param1 = -1;
              msg_vehicle_command.param2 = 1;
              msg_vehicle_command.param3 = 0;
              msg_vehicle_command.param4 = msg->yaw;
              msg_vehicle_command.param5 = msg->latitude;
              msg_vehicle_command.param6 = msg->longitude;
              msg_vehicle_command.param7 = msg->altitude;
              msg_vehicle_command.confirmation = 0;
              msg_vehicle_command.source_system = 255;
              msg_vehicle_command.target_system = target_system_;
              msg_vehicle_command.target_component = 1;
              msg_vehicle_command.from_external = true;
              vehicle_command_pub_->publish(msg_vehicle_command);
      });

  vehicle_gps_sensor_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("gps", 10);
  vehicle_gps_position_sub_ = create_subscription<px4_msgs::msg::VehicleGpsPosition>(
            "vehicle_gps_position",
            10,
            [this](px4_msgs::msg::VehicleGpsPosition::ConstSharedPtr msg){
              if(geodetic_converter==nullptr){
                geodetic_converter = std::make_shared<GeodeticConverter>(msg->lat*1E-7, msg->lon*1E-7, msg->alt*1E-3);
              }

              latitude_ = msg->lat*1e-7;
              longitude_ = msg->lon*1e-7;

              sensor_msgs::msg::NavSatFix msg_gps;
              msg_gps.header.stamp =  get_clock()->now();
              msg_gps.latitude = msg->lat*1e-7;
              msg_gps.longitude = msg->lon*1e-7;
              msg_gps.altitude = msg->alt*1e-3;
              msg_gps.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
              if(msg->fix_type == 0 || msg->fix_type == 1){
                msg_gps.status.status = -1;//sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
              }else if(msg->fix_type == 2 || msg->fix_type == 3){
                msg_gps.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
              }
              vehicle_gps_sensor_pub_->publish(msg_gps);
            });

  vehicle_attitude_pub_ = create_publisher<proposed_aerial_msgs::msg::Attitude>("attitude", 10);
  vehicle_atitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
            "vehicle_attitude",
            10,
            [this](px4_msgs::msg::VehicleAttitude::ConstSharedPtr msg) {
              proposed_aerial_msgs::msg::Attitude msg_to_send;
              msg_to_send.header.stamp = get_clock()->now();
              msg_to_send.orientation.x = msg->q[1];
              msg_to_send.orientation.y = msg->q[2];
              msg_to_send.orientation.z = msg->q[3];
              msg_to_send.orientation.w = msg->q[0];

              geometry_msgs::msg::Quaternion q;
              q.x = msg_to_send.orientation.x;
              q.y = msg_to_send.orientation.y;
              q.z = msg_to_send.orientation.z;
              q.w = msg_to_send.orientation.w;
              double yaw, pitch, roll;
              tf2::getEulerYPR(q, yaw, pitch, roll);
              heading_ = yaw;

              msg_to_send.reference_frame_type = proposed_aerial_msgs::msg::Attitude::ENU;

              vehicle_attitude_pub_->publish(msg_to_send);
            });

  pose_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "command_pose",
            10,
            [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {

              double lat, lon, alt;
              double h_lat, h_lon, h_alt;

              if(geodetic_converter==nullptr)
                return;

              geodetic_converter->getHome(h_lat, h_lon, h_alt);

              double east, north, up;
              geodetic_converter->geodetic2Enu(h_lat, h_lon, h_alt,
                                              east, north, up);

              east += msg->pose.position.x;
              north += msg->pose.position.y;
              geodetic_converter->enu2Geodetic(east, north, up, lat, lon, alt);

              RCLCPP_INFO(get_logger(),
                              "lat %.5f\tlon %.5f\talt: %.5f", lat, lon, alt);

              geometry_msgs::msg::Quaternion q;
              q.x = msg->pose.orientation.x;
              q.y = msg->pose.orientation.y;
              q.z = msg->pose.orientation.z;
              q.w = msg->pose.orientation.w;
              double yaw, pitch, roll;
              tf2::getEulerYPR(q, yaw, pitch, roll);

              px4_msgs::msg::VehicleCommand msg_vehicle_command;
              msg_vehicle_command.timestamp = get_clock()->now().nanoseconds()/1000;
              msg_vehicle_command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_REPOSITION;
              msg_vehicle_command.param1 = -1;
              msg_vehicle_command.param2 = 1;
              msg_vehicle_command.param3 = 0;
              msg_vehicle_command.param4 = -yaw+1.57;
              msg_vehicle_command.param5 = lat;
              msg_vehicle_command.param6 = lon;
              msg_vehicle_command.param7 = msg->pose.position.z;
              msg_vehicle_command.confirmation = 0;
              msg_vehicle_command.source_system = 255;
              msg_vehicle_command.target_system = target_system_;
              msg_vehicle_command.target_component = 1;
              msg_vehicle_command.from_external = true;
              vehicle_command_pub_->publish(msg_vehicle_command);

      });

  action_flight_mode_server_ = rclcpp_action::create_server<proposed_aerial_msgs::action::SetFlightMode>(
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "set_flight_mode",
      std::bind(&DroneNode::handle_flight_mode_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&DroneNode::handle_flight_mode_cancel, this, std::placeholders::_1),
      std::bind(&DroneNode::handle_flight_mode_accepted, this, std::placeholders::_1));

}

rclcpp_action::GoalResponse  DroneNode::handle_flight_mode_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const proposed_aerial_msgs::action::SetFlightMode::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->goal.flight_mode);
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse  DroneNode::handle_flight_mode_cancel(
   const std::shared_ptr<rclcpp_action::ServerGoalHandle<proposed_aerial_msgs::action::SetFlightMode>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void  DroneNode::execute_flight_mode(const std::shared_ptr<rclcpp_action::ServerGoalHandle<proposed_aerial_msgs::action::SetFlightMode>> goal_handle)
{

  RCLCPP_INFO(this->get_logger(), "execute_flight_mode");

  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<proposed_aerial_msgs::action::SetFlightMode::Feedback>();
  auto result = std::make_shared<proposed_aerial_msgs::action::SetFlightMode::Result>();
  rclcpp::Rate loop_rate(100);
  px4_msgs::msg::VehicleCommand msg_vehicle_command;

  if(goal->goal.flight_mode == proposed_aerial_msgs::msg::FlightMode::FLIGHT_MODE_ARMED ||
      goal->goal.flight_mode == proposed_aerial_msgs::msg::FlightMode::FLIGHT_MODE_DISARMED){
      //ARM
      msg_vehicle_command.timestamp = get_clock()->now().nanoseconds()/1000;
      msg_vehicle_command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
      msg_vehicle_command.param1 = (goal->goal.flight_mode == proposed_aerial_msgs::msg::FlightMode::FLIGHT_MODE_ARMED);
      msg_vehicle_command.confirmation = 1;
      msg_vehicle_command.source_system = 255;
      msg_vehicle_command.target_system = target_system_;
      msg_vehicle_command.target_component = 1;
      msg_vehicle_command.from_external = true;
  }
  if(goal->goal.flight_mode == proposed_aerial_msgs::msg::FlightMode::FLIGHT_MODE_FLYING){
    msg_vehicle_command.timestamp = get_clock()->now().nanoseconds()/1000;
    msg_vehicle_command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF;
    msg_vehicle_command.param1 = 0.1;
    msg_vehicle_command.param2 = 0;
    msg_vehicle_command.param3 = 0;
    msg_vehicle_command.param4 = heading_;
    msg_vehicle_command.param5 = latitude_;
    msg_vehicle_command.param6 = longitude_;
    msg_vehicle_command.param7 = 3.0;
    msg_vehicle_command.confirmation = 1;
    msg_vehicle_command.source_system = 255;
    msg_vehicle_command.target_system = target_system_;
    msg_vehicle_command.target_component = 1;
    msg_vehicle_command.from_external = true;
  }
  if(goal->goal.flight_mode == proposed_aerial_msgs::msg::FlightMode::FLIGHT_MODE_LANDED){
    msg_vehicle_command.timestamp = get_clock()->now().nanoseconds()/1000;
    msg_vehicle_command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND;
    msg_vehicle_command.param1 = 0.1;
    msg_vehicle_command.param4 = heading_;
    msg_vehicle_command.param5 = latitude_;
    msg_vehicle_command.param6 = longitude_;
    msg_vehicle_command.confirmation = 1;
    msg_vehicle_command.source_system = 255;
    msg_vehicle_command.target_system = target_system_;
    msg_vehicle_command.target_component = 1;
    msg_vehicle_command.from_external = true;
  }

  //TODO(ahcorde): find a way to spin, because we need to arming_state_ updated
  int initial_arming_state = arming_state_;
  int initial_flying = flying_;
  vehicle_command_pub_->publish(msg_vehicle_command);

  auto time_point_init = get_clock()->now().seconds();
  while(initial_arming_state == arming_state_ && initial_flying == flying_ &&
     (get_clock()->now().seconds() - time_point_init) < 25){

       // Check if there is a cancel request
       if (goal_handle->is_canceling()) {
         result->success = false;
         goal_handle->canceled(result);
         RCLCPP_INFO(this->get_logger(), "Goal Canceled");
         return;
       }
       if(arming_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_STANDBY){
         feedback->state.flight_mode = proposed_aerial_msgs::msg::FlightMode::FLIGHT_MODE_DISARMED;
       }
       if(arming_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED){
         feedback->state.flight_mode = proposed_aerial_msgs::msg::FlightMode::FLIGHT_MODE_ARMED;
       }
       if(flying_){
         feedback->state.flight_mode = proposed_aerial_msgs::msg::FlightMode::FLIGHT_MODE_FLYING;
       }
       goal_handle->publish_feedback(feedback);
       loop_rate.sleep();
  }


  if(goal->goal.flight_mode == proposed_aerial_msgs::msg::FlightMode::FLIGHT_MODE_ARMED ||
      goal->goal.flight_mode == proposed_aerial_msgs::msg::FlightMode::FLIGHT_MODE_DISARMED){
    if(goal->goal.flight_mode == proposed_aerial_msgs::msg::FlightMode::FLIGHT_MODE_ARMED){
        if(arming_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED){
          result->success = true;
        }
    }else if(goal->goal.flight_mode == proposed_aerial_msgs::msg::FlightMode::FLIGHT_MODE_DISARMED){
        if(arming_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_STANDBY){
          result->success = true;
        }
    }
  }

  if(goal->goal.flight_mode == proposed_aerial_msgs::msg::FlightMode::FLIGHT_MODE_LANDED){
    result->success = flying_;
  }
  if(goal->goal.flight_mode == proposed_aerial_msgs::msg::FlightMode::FLIGHT_MODE_FLYING){
    result->success = !flying_;
  }

  if(arming_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_STANDBY){
    result->result.flight_mode = proposed_aerial_msgs::msg::FlightMode::FLIGHT_MODE_DISARMED;
  }
  if(arming_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED){
    result->result.flight_mode = proposed_aerial_msgs::msg::FlightMode::FLIGHT_MODE_ARMED;
  }
  if(flying_){
    result->result.flight_mode = proposed_aerial_msgs::msg::FlightMode::FLIGHT_MODE_FLYING;
  }

  if (rclcpp::ok()) {
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
  }

}

void  DroneNode::handle_flight_mode_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<proposed_aerial_msgs::action::SetFlightMode>> goal_handle)
{
  std::thread{std::bind(&DroneNode::execute_flight_mode, this, std::placeholders::_1), goal_handle}.detach();
}

void DroneNode::flight_mode_timer_callback()
{
  proposed_aerial_msgs::msg::FlightMode msg_to_send;

  if( arming_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_STANDBY){
    msg_to_send.flight_mode = proposed_aerial_msgs::msg::FlightMode::FLIGHT_MODE_DISARMED;
  }else if(!flying_ && arming_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED){
    msg_to_send.flight_mode = proposed_aerial_msgs::msg::FlightMode::FLIGHT_MODE_ARMED;
  }else if (flying_){
    if(nav_state_ == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND){
      msg_to_send.flight_mode = proposed_aerial_msgs::msg::FlightMode::FLIGHT_MODE_LANDED;
    }else if(nav_state_ == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_RTL){
      msg_to_send.flight_mode = proposed_aerial_msgs::msg::FlightMode::FLIGHT_MODE_RTL;
    }else{
      msg_to_send.flight_mode = proposed_aerial_msgs::msg::FlightMode::FLIGHT_MODE_FLYING;
    }
  }
  flight_mode_pub_->publish(std::move(msg_to_send));
}
//
// void DroneNode::set_fligh_mode_handle_service(
//   const std::shared_ptr<rmw_request_id_t> request_header,
//   const std::shared_ptr<proposed_aerial_msgs::action::SetFlightMode::Request> request,
//   const std::shared_ptr<proposed_aerial_msgs::action::SetFlightMode::Response> response)
// {
//   (void)request_header;
//   px4_msgs::msg::VehicleCommand msg_vehicle_command;
//
//   if(request->goal.flight_mode == proposed_aerial_msgs::msg::FlightMode::FLIGHT_MODE_ARMED ||
//      request->goal.flight_mode == proposed_aerial_msgs::msg::FlightMode::FLIGHT_MODE_DISARMED){
//     //ARM
//     msg_vehicle_command.timestamp = get_clock()->now().nanoseconds()/1000;
//     msg_vehicle_command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
//     msg_vehicle_command.param1 = (request->goal.flight_mode == proposed_aerial_msgs::msg::FlightMode::FLIGHT_MODE_ARMED);
//     msg_vehicle_command.confirmation = 1;
//     msg_vehicle_command.source_system = 255;
//     msg_vehicle_command.target_system = target_system_;
//     msg_vehicle_command.target_component = 1;
//     msg_vehicle_command.from_external = true;
//     vehicle_command_pub_->publish(msg_vehicle_command);
//
//     //TODO(ahcorde): find a way to spin, because we need to arming_state_ updated
//     // int initial_arming_state = arming_state_;
//     //
//     // auto time_point_init = get_clock()->now().seconds();
//     // while(initial_arming_state == arming_state_ &&
//     //    (get_clock()->now().seconds() - time_point_init) < 5){
//     //      std::this_thread::sleep_for (std::chrono::milliseconds(10));
//     // }
//
//     if(request->goal.flight_mode == proposed_aerial_msgs::msg::FlightMode::FLIGHT_MODE_ARMED){
//       if(arming_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED){
//         response->success = true;
//       }
//     }else if(request->goal.flight_mode == proposed_aerial_msgs::msg::FlightMode::FLIGHT_MODE_DISARMED){
//       if(arming_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_STANDBY){
//         response->success = true;
//      }
//    }
//   }
//   if(request->goal.flight_mode == proposed_aerial_msgs::msg::FlightMode::FLIGHT_MODE_FLYING){
//     msg_vehicle_command.timestamp = get_clock()->now().nanoseconds()/1000;
//     msg_vehicle_command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF;
//     msg_vehicle_command.param1 = 0.1;
//     msg_vehicle_command.param2 = 0;
//     msg_vehicle_command.param3 = 0;
//     msg_vehicle_command.param4 = heading_;
//     msg_vehicle_command.param5 = latitude_;
//     msg_vehicle_command.param6 = longitude_;
//     msg_vehicle_command.param7 = 3.0;
//     msg_vehicle_command.confirmation = 1;
//     msg_vehicle_command.source_system = 255;
//     msg_vehicle_command.target_system = target_system_;
//     msg_vehicle_command.target_component = 1;
//     msg_vehicle_command.from_external = true;
//     vehicle_command_pub_->publish(msg_vehicle_command);
//
//     response->success = flying_;
//
//   }
//   if(request->goal.flight_mode == proposed_aerial_msgs::msg::FlightMode::FLIGHT_MODE_LANDED){
//     msg_vehicle_command.timestamp = get_clock()->now().nanoseconds()/1000;
//     msg_vehicle_command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND;
//     msg_vehicle_command.param1 = 0.1;
//     msg_vehicle_command.param4 = heading_;
//     msg_vehicle_command.param5 = latitude_;
//     msg_vehicle_command.param6 = longitude_;
//     msg_vehicle_command.confirmation = 1;
//     msg_vehicle_command.source_system = 255;
//     msg_vehicle_command.target_system = target_system_;
//     msg_vehicle_command.target_component = 1;
//     msg_vehicle_command.from_external = true;
//     vehicle_command_pub_->publish(msg_vehicle_command);
//
//     response->success = !flying_;
//   }
//
//
//   if(arming_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_STANDBY){
//    response->result.flight_mode = proposed_aerial_msgs::msg::FlightMode::FLIGHT_MODE_DISARMED;
//   }
//   if(arming_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED){
//    response->result.flight_mode = proposed_aerial_msgs::msg::FlightMode::FLIGHT_MODE_ARMED;
//   }
//   if(flying_){
//     response->result.flight_mode = proposed_aerial_msgs::msg::FlightMode::FLIGHT_MODE_FLYING;
//   }
// }

DroneNode::~DroneNode()
{

}
