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

from px4_msgs.msg import VehicleCommand, VehicleGpsPosition
from px4_msgs.msg import VehicleStatus, VehicleOdometry, VehicleLandDetected
import rclpy
from rclpy.node import Node

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

        self.vehicle_command_pub = self.create_publisher(VehicleCommand,
                                                         self.get_namespace() +
                                                         '/vehicle_command',
                                                         1)

        self.vehicle_status_sub = self.create_subscription(VehicleStatus,
                                                       self.get_namespace() +
                                                       '/vehicle_status',
                                                       self.statusCallback, 10)

        self.model_pose_sub = self.create_subscription(VehicleGpsPosition,
                                                       self.get_namespace() +
                                                       '/vehicle_gps_position',
                                                       self.poseCallback, 10)

        self.vehicle_odometry_sub = self.create_subscription(VehicleOdometry,
                                                       self.get_namespace() +
                                                       '/vehicle_odometry',
                                                       self.odometryCallback, 10)

        self.vehicle_land_sub = self.create_subscription(VehicleLandDetected,
                                                       self.get_namespace() +
                                                       '/vehicle_land_detected',
                                                       self.landCallback, 10)

        self.initial_pose_received = False
        self.current_pose = None

        self.arming_state = -1
        self.relative_alt = 0
        self.landed = -1

    def arm_vehicle(self, timeout=5, attempts_limit=5):
        attempt = 0
        while self.arming_state != VehicleStatus.ARMING_STATE_ARMED:
            attempt = attempt + 1
            self.info_msg('Trying to arm vehicle')
            msg_vehicle_command = VehicleCommand()
            msg_vehicle_command.timestamp = int(self.get_clock().now().nanoseconds/1000.0)
            msg_vehicle_command.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
            msg_vehicle_command.param1 = 1.0
            msg_vehicle_command.param2 = 0.0
            msg_vehicle_command.param3 = 0.0
            msg_vehicle_command.param4 = 0.0
            msg_vehicle_command.param5 = 0.0
            msg_vehicle_command.param6 = 0.0
            msg_vehicle_command.param7 = 0.0
            msg_vehicle_command.confirmation = 1
            msg_vehicle_command.source_system = 255
            msg_vehicle_command.target_system = self.target_system
            msg_vehicle_command.target_component = 1
            msg_vehicle_command.from_external = True
            print(self.vehicle_command_pub.topic)
            self.vehicle_command_pub.publish(msg_vehicle_command)

            start_time = time.time()
            while (time.time() - start_time) < timeout:
                rclpy.spin_once(self, timeout_sec=1)
            if (attempt > attempts_limit):
                return False
        self.info_msg('Vehicle armed')
        return True

    def takeoff_vehicle(self, timeout=5, attempts_limit=5):
        attempt = 0
        while not self.initial_pose_received:
            rclpy.spin_once(self, timeout_sec=0.1)
        while self.relative_alt < 2:
            attempt = attempt + 1
            self.info_msg('Trying to takeoff vehicle')
            msg_vehicle_command = VehicleCommand()
            msg_vehicle_command.timestamp = int(self.get_clock().now().nanoseconds/1000.0)
            msg_vehicle_command.command = VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF
            msg_vehicle_command.param1 = 0.1
            msg_vehicle_command.param2 = 0.0
            msg_vehicle_command.param3 = 0.0
            msg_vehicle_command.param4 = 90.0*3.1416/180
            msg_vehicle_command.param5 = self.reference_pose[0]
            msg_vehicle_command.param6 = self.reference_pose[1]
            msg_vehicle_command.param7 = 3.0
            msg_vehicle_command.confirmation = 1
            msg_vehicle_command.source_system = 255
            msg_vehicle_command.target_system = self.target_system
            msg_vehicle_command.target_component = 1
            msg_vehicle_command.from_external = True
            self.vehicle_command_pub.publish(msg_vehicle_command)
            start_time = time.time()
            while (time.time() - start_time) < timeout:
                rclpy.spin_once(self, timeout_sec=1)
            if (attempt > attempts_limit):
                return False
        self.info_msg('Vehicle flying!')
        return True

    def RTL_vehicle(self, timeout=5, attempts_limit=5):
        self.goal_pose[0] = 0
        self.goal_pose[1] = 0
        self.goal_pose[2] = 0
        self.info_msg('Trying to return to launch')
        msg_vehicle_command = VehicleCommand()
        msg_vehicle_command.timestamp = int(self.get_clock().now().nanoseconds/1000.0)
        msg_vehicle_command.command = VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH
        msg_vehicle_command.confirmation = 1
        msg_vehicle_command.source_system = 255
        msg_vehicle_command.target_system = self.target_system
        msg_vehicle_command.target_component = 1
        msg_vehicle_command.from_external = True
        self.vehicle_command_pub.publish(msg_vehicle_command)

    def info_msg(self, msg: str):
        self.get_logger().info('\033[1;37;44m' + msg + '\033[0m')

    def warn_msg(self, msg: str):
        self.get_logger().warn('\033[1;37;43m' + msg + '\033[0m')

    def error_msg(self, msg: str):
        self.get_logger().error('\033[1;37;41m' + msg + '\033[0m')

    def setGoalPose(self):
        ned3 = (self.goal_pose[0], self.goal_pose[1], -self.goal_pose[2])
        lat, lon, alt = pm.ned2geodetic(*ned3, *self.reference_pose)
        print("Flying to: ")
        print(lat, lon, alt)
        msg_vehicle_command = VehicleCommand()
        msg_vehicle_command.timestamp = int(self.get_clock().now().nanoseconds/1000.0)
        msg_vehicle_command.command = VehicleCommand.VEHICLE_CMD_DO_REPOSITION
        msg_vehicle_command.param1 = -1.0
        msg_vehicle_command.param2 = 1.0
        msg_vehicle_command.param3 = 0.0
        msg_vehicle_command.param4 = float(self.goal_pose[3]*3.1416/180)
        msg_vehicle_command.param5 = float(lat)
        msg_vehicle_command.param6 = float(lon)
        msg_vehicle_command.param7 = float(self.goal_pose[2])
        msg_vehicle_command.confirmation = 0
        msg_vehicle_command.source_system = 255
        msg_vehicle_command.target_system = self.target_system
        msg_vehicle_command.target_component = 1
        msg_vehicle_command.from_external = True
        self.vehicle_command_pub.publish(msg_vehicle_command)

    def poseCallback(self, msg):
        self.current_pose = msg
        if(not self.initial_pose_received):
            self.info_msg('Received vehicle_gps_position')
            self.reference_pose = (msg.lat*1e-7, msg.lon*1e-7, msg.alt*1e-3)
            print(self.reference_pose)
        self.initial_pose_received = True

    def statusCallback(self, msg):
        self.arming_state = msg.arming_state

    def odometryCallback(self, msg):
        self.relative_alt = -msg.z

    def landCallback(self, msg):
        self.landed = msg.landed

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
        geodetic_coord = (self.current_pose.lat*1e-7,
                          self.current_pose.lon*1e-7,
                          self.current_pose.alt*1e-3)
        x, y, z = pm.geodetic2ned(*geodetic_coord, *self.reference_pose)
        d_x = self.goal_pose[0] - x
        d_y = self.goal_pose[1] - y
        d_z = self.goal_pose[2] + z
        distance = math.sqrt(d_x * d_x + d_y * d_y + d_z * d_z)
        self.info_msg('Distance from goal is: ' + str(distance))
        return distance


def test_RobotMovesToGoal(robot_tester):
    robot_tester.info_msg('Setting goal pose')
    robot_tester.setGoalPose()
    robot_tester.info_msg('Waiting 60 seconds for robot to reach goal')
    return robot_tester.reachesGoal(timeout=60, distance=0.5)

def test_ArmVehicle(robot_tester):
    return robot_tester.arm_vehicle()


def test_TakeoffVehicle(robot_tester):
    return robot_tester.takeoff_vehicle()


def test_RTLVehicle(robot_tester):
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


def fwd_pose(x, y, z, heading):
    ned3 = (x, y, -z)
    reference = (37.7332531, -119.5616378, 2800.4)
    lat, lon, alt = pm.ned2geodetic(*ned3, *reference)
    print(reference)
    print(lat, lon, alt)

    return [lat, lon, alt, heading]


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
