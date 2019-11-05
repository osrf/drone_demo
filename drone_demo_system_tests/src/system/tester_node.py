#! /usr/bin/env python3
# Copyright 2018 Intel Corporation.
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
from typing import Optional

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from px4_msgs.msg import VehicleCommand, VehicleGpsPosition

import rclpy

from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

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

        self.publisher_vehicle_command = self.create_publisher(VehicleCommand,
                                                        self.get_namespace() + '/vehicle_command', 1)

        self.model_pose_sub = self.create_subscription(VehicleGpsPosition,
                                                       self.get_namespace() + '/vehicle_gps_position',
                                                       self.poseCallback, 10)
        self.initial_pose_received = False
        self.current_pose = None

        while not self.initial_pose_received:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.arm_vehicle()
        time.sleep(2)
        self.takeoff_vehicle()
        time.sleep(5)

    def arm_vehicle(self):
        msg_vehicle_command = VehicleCommand();
        msg_vehicle_command.timestamp = int(self.get_clock().now().nanoseconds/1000.0);
        msg_vehicle_command.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM;
        msg_vehicle_command.param1 = 1.0;
        msg_vehicle_command.param2 = 0.0;
        msg_vehicle_command.param3 = 0.0;
        msg_vehicle_command.param4 = 0.0;
        msg_vehicle_command.param5 = 0.0;
        msg_vehicle_command.param6 = 0.0;
        msg_vehicle_command.param7 = 0.0;
        msg_vehicle_command.confirmation = 1;
        msg_vehicle_command.source_system = 255;
        msg_vehicle_command.target_system = self.target_system
        msg_vehicle_command.target_component = 1;
        msg_vehicle_command.from_external = True;
        print(self.publisher_vehicle_command.topic)
        self.publisher_vehicle_command.publish(msg_vehicle_command);

    def takeoff_vehicle(self):
        while not self.initial_pose_received:
            rclpy.spin_once(self, timeout_sec=0.1)
        msg_vehicle_command = VehicleCommand();
        msg_vehicle_command.timestamp = int(self.get_clock().now().nanoseconds/1000.0);
        msg_vehicle_command.command = VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF;
        msg_vehicle_command.param1 = 0.1;
        msg_vehicle_command.param2 = 0.0;
        msg_vehicle_command.param3 = 0.0;
        msg_vehicle_command.param4 = 90.0*3.1416/180; #TODO(ahcorde):
        msg_vehicle_command.param5 = self.reference_pose[0];
        msg_vehicle_command.param6 = self.reference_pose[1];
        msg_vehicle_command.param7 = 3.0;
        msg_vehicle_command.confirmation = 1;
        msg_vehicle_command.source_system = 255;
        msg_vehicle_command.target_system = self.target_system
        msg_vehicle_command.target_component = 1;
        msg_vehicle_command.from_external = True;
        self.publisher_vehicle_command.publish(msg_vehicle_command);

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
        msg_vehicle_command = VehicleCommand();
        msg_vehicle_command.timestamp = int(self.get_clock().now().nanoseconds/1000.0);
        msg_vehicle_command.command = VehicleCommand.VEHICLE_CMD_DO_REPOSITION;
        msg_vehicle_command.param1 = -1.0;
        msg_vehicle_command.param2 = 1.0;
        msg_vehicle_command.param3 = 0.0;
        msg_vehicle_command.param4 = float(self.goal_pose[3]*3.1416/180);
        msg_vehicle_command.param5 = float(lat);
        msg_vehicle_command.param6 = float(lon);
        msg_vehicle_command.param7 = float(self.goal_pose[2]);
        msg_vehicle_command.confirmation = 0;
        msg_vehicle_command.source_system = 255;
        msg_vehicle_command.target_system = self.target_system
        msg_vehicle_command.target_component = 1;
        msg_vehicle_command.from_external = True;
        self.publisher_vehicle_command.publish(msg_vehicle_command);

    def poseCallback(self, msg):
        self.info_msg('Received vehicle_gps_position')
        self.current_pose = msg
        if(not self.initial_pose_received):
            self.reference_pose = (msg.lat*1e-7, msg.lon*1e-7, msg.alt*1e-3)
            print(self.reference_pose)
        self.initial_pose_received = True

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

    def distanceFromGoal(self):
        geodetic_coord = (self.current_pose.lat*1e-7, self.current_pose.lon*1e-7, self.current_pose.alt*1e-3)
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

def run_all_tests(robot_tester):
    # set transforms to use_sim_time
    result = True
    if (result):
        result = test_RobotMovesToGoal(robot_tester)

    if (result):
        robot_tester.info_msg('Test PASSED')
    else:
        robot_tester.error_msg('Test FAILED')

    return result

# TODO(ahcorde): remove this once tf.transformation is available
# axis sequences for Euler angles
_NEXT_AXIS = [1, 2, 0, 1]

# map axes strings to/from tuples of inner axis, parity, repetition, frame
_AXES2TUPLE = {
    'sxyz': (0, 0, 0, 0), 'sxyx': (0, 0, 1, 0), 'sxzy': (0, 1, 0, 0),
    'sxzx': (0, 1, 1, 0), 'syzx': (1, 0, 0, 0), 'syzy': (1, 0, 1, 0),
    'syxz': (1, 1, 0, 0), 'syxy': (1, 1, 1, 0), 'szxy': (2, 0, 0, 0),
    'szxz': (2, 0, 1, 0), 'szyx': (2, 1, 0, 0), 'szyz': (2, 1, 1, 0),
    'rzyx': (0, 0, 0, 1), 'rxyx': (0, 0, 1, 1), 'ryzx': (0, 1, 0, 1),
    'rxzx': (0, 1, 1, 1), 'rxzy': (1, 0, 0, 1), 'ryzy': (1, 0, 1, 1),
    'rzxy': (1, 1, 0, 1), 'ryxy': (1, 1, 1, 1), 'ryxz': (2, 0, 0, 1),
    'rzxz': (2, 0, 1, 1), 'rxyz': (2, 1, 0, 1), 'rzyz': (2, 1, 1, 1)}

_TUPLE2AXES = dict((v, k) for k, v in _AXES2TUPLE.items())

def quaternion_from_euler(ai, aj, ak, axes='sxyz'):
    """Return quaternion from Euler angles and axis sequence.
    ai, aj, ak : Euler's roll, pitch and yaw angles
    axes : One of 24 axis sequences as string or encoded tuple
    >>> q = quaternion_from_euler(1, 2, 3, 'ryxz')
    >>> numpy.allclose(q, [0.310622, -0.718287, 0.444435, 0.435953])
    True
    """
    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
    except (AttributeError, KeyError):
        _ = _TUPLE2AXES[axes]
        firstaxis, parity, repetition, frame = axes

    i = firstaxis
    j = _NEXT_AXIS[i+parity]
    k = _NEXT_AXIS[i-parity+1]

    if frame:
        ai, ak = ak, ai
    if parity:
        aj = -aj

    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    quaternion = numpy.empty((4, ), dtype=numpy.float64)
    if repetition:
        quaternion[i] = cj*(cs + sc)
        quaternion[j] = sj*(cc + ss)
        quaternion[k] = sj*(cs - sc)
        quaternion[3] = cj*(cc - ss)
    else:
        quaternion[i] = cj*sc - sj*cs
        quaternion[j] = cj*ss + sj*cc
        quaternion[k] = cj*cs - sj*sc
        quaternion[3] = cj*cc + sj*ss
    if parity:
        quaternion[j] *= -1

    return quaternion

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
            goal_pose=fwd_pose(float(final_x), float(final_y),
                            float(final_z), float(final_heading)))
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

    # wait a few seconds to make sure entire stacks are up
    # time.sleep(10)

    # run tests on each robot
    for tester in testers:
        passed = run_all_tests(tester)
        if not passed:
            exit(1)


if __name__ == '__main__':
    main()
