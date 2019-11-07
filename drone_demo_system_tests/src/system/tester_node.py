#! /usr/bin/env python3
# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import math
import numpy
import sys
import time
import numpy as np

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from proposed_aerial_msgs.action import SetFlightMode
from proposed_aerial_msgs.msg import FlightMode

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

import pymap3d as pm


class DroneTester(Node):

    def __init__(
        self,
        goal_pose: [0, 0, 0, 0],
    ):
        super().__init__(node_name='drone_tester')

        self.target_system = 1
        self.goal_pose = goal_pose

        print("pose to reach: ", self.goal_pose)

        self.set_flight_mode_action_client = ActionClient(self, SetFlightMode, 'set_flight_mode')

        self.action_response_received = False
        self.action_response_success = False
        self.action_response_result = None

        self.vehicle_command_pub = self.create_publisher(PoseStamped,
                                                         self.get_namespace() +
                                                         '/command_pose',
                                                         1)

        self.model_pose_sub = self.create_subscription(NavSatFix,
                                                       self.get_namespace() +
                                                       '/gps',
                                                       self.poseCallback, 10)

        self.flight_mode_sub = self.create_subscription(FlightMode,
                                                       self.get_namespace() +
                                                       '/flight_mode',
                                                       self.flightModeCallback, 10)

        self.initial_pose_received = False
        self.current_pose = None

        self.altitude_ref = 0

        self.landed = -1

    def send_goal(self, mode):
        self.get_logger().info('Waiting for action server...')
        self.set_flight_mode_action_client.wait_for_server()

        goal_msg = SetFlightMode.Goal()
        goal_msg.goal.flight_mode = mode

        self.get_logger().info('Sending goal request...')

        self._send_goal_future = self.set_flight_mode_action_client.send_goal_async(
            goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        self.action_response_received = True
        self.action_response_success = result.success
        self.action_response_result = result.result.flight_mode
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded!')
        else:
            self.get_logger().info('Goal failed')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def arm_vehicle(self):
        self.info_msg("arming vehicle")
        self.send_goal(FlightMode.FLIGHT_MODE_ARMED)
        self.action_response_received = False

        while(not self.action_response_received):
            rclpy.spin_once(self, timeout_sec=0.1)
        self.info_msg("arming vehicle response received")

        return (self.action_response_result == FlightMode.FLIGHT_MODE_ARMED)

    def takeoff_vehicle(self):
        self.info_msg("takeoff vehicle")

        while(self.current_pose == None):
            rclpy.spin_once(self, timeout_sec=0.1)

        self.altitude_ref = self.current_pose.altitude

        self.send_goal(FlightMode.FLIGHT_MODE_FLYING)
        self.action_response_received = False

        while(not self.action_response_received):
            rclpy.spin_once(self, timeout_sec=0.1)
        self.info_msg("takeoff vehicle response received")

        return (self.action_response_result == FlightMode.FLIGHT_MODE_FLYING)

    def RTL_vehicle(self):
        self.goal_pose[0] = 0
        self.goal_pose[1] = 0
        self.goal_pose[2] = 0
        self.info_msg("RTL vehicle")
        self.send_goal(FlightMode.FLIGHT_MODE_RTL)
        self.action_response_received = False

    def info_msg(self, msg: str):
        self.get_logger().info('\033[1;37;44m' + msg + '\033[0m')

    def warn_msg(self, msg: str):
        self.get_logger().warn('\033[1;37;43m' + msg + '\033[0m')

    def error_msg(self, msg: str):
        self.get_logger().error('\033[1;37;41m' + msg + '\033[0m')

    def euler_to_quaternion(self, yaw, pitch, roll):

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]

    def setGoalPose(self):
          msg = PoseStamped();
          msg.header.stamp = self.get_clock().now().to_msg();

          msg.pose.position.x = self.goal_pose[0]
          msg.pose.position.y = self.goal_pose[1]
          msg.pose.position.z = self.goal_pose[2]

          q = self.euler_to_quaternion(-self.goal_pose[3]+1.57, 0, 0)

          msg.pose.orientation.x = q[0]
          msg.pose.orientation.y = q[1];
          msg.pose.orientation.z = q[2];
          msg.pose.orientation.w = q[3];

          self.vehicle_command_pub.publish(msg)

    def poseCallback(self, msg):
        self.current_pose = msg
        if(not self.initial_pose_received):
            self.info_msg('Received vehicle_gps_position')
            self.reference_pose = (msg.latitude, msg.longitude, msg.altitude)
            print(self.reference_pose)
        self.initial_pose_received = True

    def flightModeCallback(self, msg):
        self.landed = msg.flight_mode == FlightMode.FLIGHT_MODE_DISARMED

    def reachesGoal(self, timeout, distance):
        goalReached = False
        start_time = time.time()

        while not goalReached:
            rclpy.spin_once(self, timeout_sec=1)
            if(not self.initial_pose_received):
                continue
            if self.distanceFromGoal() < distance:
                goalReached = True
                self.info_msg('*** GOAL REACHED ***')
                return True
            elif timeout is not None:
                if (time.time() - start_time) > timeout:
                    self.error_msg('Robot timed out reaching its goal!')
                    return False

    def reachesTakeOffAltitude(self, timeout, distance):
        altitudeReached = False
        start_time = time.time()

        while not altitudeReached:
            rclpy.spin_once(self, timeout_sec=1)
            if self.distanceTakeOff() > 2.5:
                altitudeReached = True
                self.info_msg('*** TakeOff altitude REACHED ***')
                return True
            elif timeout is not None:
                if (time.time() - start_time) > timeout:
                    self.error_msg('Robot timed out reaching its takeoff altitude!')
                    return False

    def reachesRTL(self, timeout, distance):
        homeReached = False
        start_time = time.time()

        while not homeReached:
            rclpy.spin_once(self, timeout_sec=1)
            if self.landed and self.distanceFromGoal() < distance:
                homeReached = True
                self.info_msg('*** RTL REACHED ***')
                return True
            elif timeout is not None:
                if (time.time() - start_time) > timeout:
                    self.error_msg('Robot timed out reaching its RTL!')
                    return False

    def distanceFromGoal(self):
        geodetic_coord = (self.current_pose.latitude,
                          self.current_pose.longitude,
                          self.current_pose.altitude)
        x, y, z = pm.geodetic2ned(*geodetic_coord, *self.reference_pose)
        d_x = self.goal_pose[0] - x
        d_y = self.goal_pose[1] - y
        d_z = self.goal_pose[2] + z
        distance = math.sqrt(d_x * d_x + d_y * d_y + d_z * d_z)
        self.info_msg('Distance from goal is: ' + str(distance))
        return distance

    def distanceTakeOff(self):
        return self.current_pose.altitude - self.altitude_ref

def test_RobotMovesToGoal(robot_tester):
    robot_tester.info_msg('Setting goal pose')
    robot_tester.setGoalPose()
    robot_tester.info_msg('Waiting 60 seconds for robot to reach goal')
    return robot_tester.reachesGoal(timeout=60, distance=0.5)


def test_ArmVehicle(robot_tester):
    return robot_tester.arm_vehicle()


def test_TakeoffVehicle(robot_tester):
    robot_tester.takeoff_vehicle()
    return robot_tester.reachesTakeOffAltitude(timeout=60, distance=0.5)


def test_RTLVehicle(robot_tester):
    robot_tester.info_msg('RTL_vehicle')
    robot_tester.RTL_vehicle()
    return robot_tester.reachesRTL(timeout=60, distance=0.5)


def run_all_tests(robot_tester):
    # set transforms to use_sim_time
    result = True
    if (result):
        resutl = test_ArmVehicle(robot_tester)
    if (result):
        result = test_TakeoffVehicle(robot_tester)
    if (result):
        result = test_RobotMovesToGoal(robot_tester)
    if (result):
        result = test_RTLVehicle(robot_tester)

    if (result):
        robot_tester.info_msg('Test PASSED')
    else:
        robot_tester.error_msg('Test FAILED')

    return result

def get_testers(args):
    testers = []

    if args.robot:
        # Requested tester for one robot
        final_x, final_y, final_z, final_heading = args.robot[0]
        tester = DroneTester(
                             goal_pose=[float(final_x), float(final_y),
                                        float(final_z), float(final_heading)])
        tester.info_msg(
            'Starting tester, robot going from ' + final_x + ', ' + final_y +
            ', ' + final_z + ', ' + final_heading + 'º.')
        testers.append(tester)
        return testers

    # Requested tester for multiple robots
    for robot in args.robots:
        namespace, init_x, init_y, final_x, final_y = robot
        tester = DroneTester(
                             namespace=namespace,
                             goal_pose=[float(final_x), float(final_y),
                                        float(final_z), float(final_heading)])
        tester.info_msg(
            'Starting tester for ' + namespace +
            ' going from ' + init_x + ', ' + init_y +
            ' to ' + final_x + ', ' + final_y)
        testers.append(tester)
    return testers


def main(argv=sys.argv[1:]):
    # The robot(s) positions from the input arguments
    parser = argparse.ArgumentParser(description='System-level drone demo tester node')
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('-r', '--robot', action='append', nargs=4,
                       metavar=('final_x', 'final_y', 'final_z', 'final_heading'),
                       help='The robot final position.')
    group.add_argument('-rs', '--robots', action='append', nargs=5,
                       metavar=('name', 'final_x', 'final_y', 'final_z', 'final_heading'),
                       help="The robot's namespace and final position. " +
                            'Repeating the argument for multiple robots is supported.')

    args, unknown = parser.parse_known_args()

    rclpy.init()

    # Create testers for each robot
    testers = get_testers(args)

    # run tests on each robot
    for tester in testers:
        passed = run_all_tests(tester)
        if not passed:
            exit(1)


if __name__ == '__main__':
    main()
