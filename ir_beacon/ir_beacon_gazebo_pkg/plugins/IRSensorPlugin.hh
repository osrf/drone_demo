/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#ifndef IR_BEACON_PLUGINS_IRSENSORPLUGIN_HH_
#define IR_BEACON_PLUGINS_IRSENSORPLUGIN_HH_

#include <memory>
#include <set>
#include <string>

#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/rendering/Camera.hh>

#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>

#include <rclcpp/rclcpp.hpp>
#include <ir_beacon_msgs/msg/targets.hpp>

namespace gazebo
{
  class IRMaterialHandler;

  /// \def IRMaterialHandlerPtr
  /// \brief Shared pointer to a IRMaterialHandlerPtr object
  using IRMaterialHandlerPtr = std::shared_ptr<IRMaterialHandler>;

  /// \brief A sensor plugin that can turn a normal camera into an IR camera
  class GAZEBO_VISIBLE IRSensorPlugin : public SensorPlugin
  {
    /// \brief Constructor
    public: IRSensorPlugin();

    /// \brief Destructor
    public: virtual ~IRSensorPlugin();

    // Documentation inherited
    public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    /// \brief Callback for new IR targets are specified
    /// \param[in] _msg Targets message containing a string array field
    /// named 'targets'
    public: void TargetCallback(const ir_beacon_msgs::msg::Targets::SharedPtr _msg);

    /// \brief Pointer to parent sensor
    private: sensors::CameraSensorPtr parentSensor;

    /// \brief Pointer to parent sensor's camera
    private: rendering::CameraPtr camera;

    /// \brief switcher the material of objects in camera view for simulating
    /// IR camera
    private: IRMaterialHandlerPtr materialHandler;

    /// \brief ROS node handle
    private: gazebo_ros::Node::SharedPtr ros_node_;

    /// \brief ROS subscriber to IR target topic
    private: rclcpp::Subscription<ir_beacon_msgs::msg::Targets>::SharedPtr sub_;
  };
}
#endif
