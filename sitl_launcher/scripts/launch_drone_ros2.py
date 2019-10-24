#!/usr/bin/env python3

import argparse
try:
    from cStringIO import StringIO
except ImportError:
    from io import StringIO
from contextlib2 import ExitStack
from distutils.dir_util import copy_tree
import math
import os
import re
import subprocess
import sys
import tempfile

# from em import Interpreter
from gazebo_msgs.srv import DeleteModel
# from gazebo_msgs.srv import DeleteModelRequest
from gazebo_msgs.srv import SpawnEntity
# from gazebo_msgs.srv import SpawnModelRequest
from ament_index_python.packages import get_package_share_directory, get_package_prefix

# from rosgraph import ROS_MASTER_URI
# from rospkg import RosPack
# from rospy import ServiceProxy
# from rospy import myargv as rospy_myargv
# from rospy import wait_for_service
import rclpy

def spawn_model(node, model_name, model_xml,
    pose, ros_master_uri=None, robot_namespace=None, debug=True,
    service_name='/spawn_entity',
):
    x, y, yaw = pose
    INITIAL_HEIGHT = 0.1 # m

    cli = node.create_client(SpawnEntity, service_name)

    while not cli.wait_for_service(timeout_sec=1.0):
        print('service not available, waiting again...')

    if debug:
        print(model_xml)

    req = SpawnEntity.Request()
    req.name = model_name
    req.xml = model_xml
    req.robot_namespace = robot_namespace if robot_namespace else model_name
    req.initial_pose.position.x = x
    req.initial_pose.position.y = y
    req.initial_pose.position.z = INITIAL_HEIGHT
    req.initial_pose.orientation.x = 0.0
    req.initial_pose.orientation.y = 0.0
    req.initial_pose.orientation.z = math.sin(yaw / 2.0)
    req.initial_pose.orientation.w = math.cos(yaw / 2.0)
    req.reference_frame = ''
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    resp = future.result()
    if resp is not None:
        print(resp.status_message, '(%s)' % model_name)
        return 0
    else:
        print(resp.status_message, file=sys.stderr)
        return 1
#
# def delete_model(model_name, ros_master_uri=None, robot_namespace=None,
#     service_name='/gazebo/delete_model'):
#
#     if ros_master_uri:
#         original_uri = os.environ[ROS_MASTER_URI]
#         os.environ[ROS_MASTER_URI] = ros_master_uri
#     wait_for_service(service_name)
#     srv = ServiceProxy(service_name, DeleteModel)
#
#     req = DeleteModelRequest()
#     req.model_name = model_name
#
#     resp = srv(req)
#
#     if ros_master_uri:
#         os.environ[ROS_MASTER_URI] = original_uri
#
#     if resp.success:
#         print(resp.status_message, '(%s)' % model_name)
#         return 0
#     else:
#         print(resp.status_message, file=sys.stderr)
#         return 1
#
#
def get_px4_dir():
    return get_package_share_directory('px4')


def seed_rootfs(rootfs):
    px4_dir = get_px4_dir()
    print("seeding rootfs at %s from %s" % (rootfs, px4_dir))
    copy_tree(px4_dir, rootfs)


def run_px4(rootfs, rc_script='etc/init.d-posix/rcS', px4_sim_model='iris', vehicle_id='0'):
    """ Replacing run_px4.sh'
    rc_script is the path to the rc_script to run
    px4_sim_model is the model
    if rootfs is set use that for the rootfs, otherwise run in a temporary directory.
    """
    if not rootfs:

            return run_px4(rc_script, px4_sim_model, tempdir)

    print("using rootfs ", rootfs)
    seed_rootfs(rootfs)

    cmd = ['bin/px4', '%s/ROMFS/px4fmu_common' % rootfs,
           '-s', rc_script,
           '-i', vehicle_id,
           '-d']
    print("running px4 with command: ", cmd)
    subprocess_env = os.environ.copy()
    subprocess_env['PX4_SIM_MODEL'] = px4_sim_model
    child = subprocess.Popen(
        cmd,
        cwd=rootfs,
        env=subprocess_env)
    return child


xacro_args = 'vehicle_name:=%(vehicle_name)s rotors_description_dir:=%(description_path)s mavlink_udp_port:=%(mavlink_udp_port)s mavlink_tcp_port:=%(mavlink_tcp_port)s camera_udp_port:=%(camera_udp_port)s camera_control_udp_port:=%(camera_control_udp_port)s camera_enable:=false --inorder > /tmp/model.urdf'
valid_models = {
    'iris': 'ros2 run xacro xacro %(description_path)s/iris/%(drone_type)s.urdf.xacro ' + xacro_args,
    'plane': 'ros2 run xacro xacro %(description_path)s/plane/%(drone_type)s.urdf.xacro ' + xacro_args,
    'typhoon_h480': 'ros2 run xacro xacro %(description_path)s/typhoon_h480/%(drone_type)s.sdf.xacro ' + xacro_args,
}

xacro_args_sdf = 'vehicle_name:=%(vehicle_name)s rotors_description_dir:=%(description_path)s mavlink_udp_port:=%(mavlink_udp_port)s mavlink_tcp_port:=%(mavlink_tcp_port)s camera_udp_port:=%(camera_udp_port)s camera_control_udp_port:=%(camera_control_udp_port)s camera_enable:=false --inorder'
valid_models_sdf = {
    'iris': 'ros2 run xacro xacro %(description_path)s/iris/%(drone_type)s.sdf.xacro ' + xacro_args_sdf,
    'plane': 'ros2 run xacro xacro %(description_path)s/plane/%(drone_type)s.sdf.xacro ' + xacro_args_sdf,
    'typhoon_h480': 'ros2 run xacro xacro %(description_path)s/typhoon_h480/%(drone_type)s.sdf.xacro ' + xacro_args_sdf,
}

starting_poses={
    '0': (0.0, 0.0, 0.0),
    '1': (0.0, 1.5, 0.0),
    '2': (0.0, 3.0, 0.0),
    '3': (0.0, 4.5, 0.0),
}

class Drone:
    def __init__(self, node, drone_type, pose=(0,0,0), vehicle_id='0'):
        self.node = node
        assert drone_type in valid_models.keys()
        self.drone_type = drone_type
        self.pose = pose
        self.vehicle_id = vehicle_id
        self.vehicle_name = self.drone_type+'_%s' % self.vehicle_id
        assert int(vehicle_id) <= 10

        description_path = os.path.join(get_package_share_directory('mavlink_sitl_gazebo'), 'models')
        self.arguments = {
            'description_path': description_path,
            'drone_type': drone_type,
            'mavlink_tcp_port': 4560 + int(vehicle_id),
            'mavlink_udp_port': 14560 + int(vehicle_id),
            'camera_control_udp_port': 14530 + int(vehicle_id),
            'camera_udp_port': 5600 + int(vehicle_id),
            'vehicle_name': self.vehicle_name,
        }

        subprocess.check_output(valid_models[drone_type] % self.arguments, shell=True).decode('utf-8')
        self.xml = subprocess.check_output(valid_models_sdf[drone_type] % self.arguments, shell=True).decode('utf-8')
        subprocess.Popen(["ros2", "run", "robot_state_publisher", "robot_state_publisher", "/tmp/model.urdf", "joint_states:=/" + self.vehicle_name + "/joint_states"])

    def spawn(self):
        spawn_model(self.node, self.vehicle_name, self.xml, self.pose)

    def unspawn(self):
        delete_model(self.vehicle_name)

    def wait(self):
        if self.autopilot_process:
            self.autopilot_process.wait()

    def __enter__(self):
        self.spawn()
        self.rootfs = tempfile.TemporaryDirectory()
        self.autopilot_process = run_px4(self.rootfs.name, 'etc/init.d-posix/rcS', self.drone_type, self.vehicle_id)
        return self

    def __exit__(self, exc_type, exc_value, exc_traceback):
        if exc_type is KeyboardInterrupt:
            print("Caught keyboard interrupt tearing down")
        print("Stopping Autopilot")
        self.autopilot_process.terminate()
        self.autopilot_process.wait()
        self.rootfs.cleanup()
        print('Cleaned up autopilot %s' % self.vehicle_id)
        print('Skipping model unload due to crashes')
        # TODO(tfoote) unloading leads to crashes that needs to be debugged
        # print('Deleting Model %s' % self.vehicle_name)
        # self.unspawn()
        # print('Finished deleting model')
        if exc_type is KeyboardInterrupt:
            return True
        # TODO(tfoote) unspawn

def run_drones(drones):
    with ExitStack() as stack:
        for id, drone in drones.items():
            print("starting %s" % id)
            stack.enter_context(drone)
        for id, drone in drones.items():
            print("waiting on %s" % id)
            drone.wait()


class DroneSelector:

    def __init__(self):
        from tkinter import Tk, Button, Label, StringVar
        from tkinter.ttk import Combobox

        self.node = rclpy.create_node('spawn_models')

        self.type_map = {'iris': 2, 'plane': 3, 'typhoon_h480': 4}

        self.window = Tk()
        self.window.title("Please select the drones you want to launch")
        self.lbl = Label(self.window, text="Please select the number of drones you want to launch.")
        self.lbl.grid(column=0, row=0, columnspan=2)
        self.btn = Button(self.window, text="Done Selecting", command=self._close_window)
        self.btn.grid(column=0, row=5, columnspan=2)

        self.labels = {}
        self.combos = {}
        self.defaults = {}
        for t, r in self.type_map.items():
            self.defaults[t] = StringVar()
            self.defaults[t].set('0')
            self.labels[t] = Label(self.window, text="%s: " %t)
            self.labels[t].grid(column=0, row=r, sticky='E')
            self.combos[t] = Combobox(self.window, textvariable=self.defaults[t], state='readonly')
            self.combos[t].grid(column=1, row=r)
            self.combos[t]['values']= (0, 1, 2, 3, 4)

    def get_drones(self):
        drones = {}
        id = 0
        for t in self.type_map.keys():
            type_combo = self.combos[t].get()
            print("Selected %s: %s" % (t, type_combo))
            type_count = int(type_combo)
            for counter in range(type_count):
                #TODO(tfoote) do better range checking to prevent
                id_str = str(id)
                if id_str not in starting_poses:
                    print("error cannot provision more drone %s, no starting pose defined" % id_str)
                    sys.exit()
                drones[id_str] = Drone(self.node, t, starting_poses[id_str], id_str)
                id += 1
        self.drones = drones

    def _close_window(self):
        self.get_drones()
        self.window.destroy()

    def mainloop(self):

        self.window.mainloop()

def main():
    SUPPORTED_DRONE_TYPES=['typhoon_h480', 'iris', 'plane']

    parser = argparse.ArgumentParser(description='Spawn a drone')
    # parser.add_argument('drone_type', help="What type of drone", choices=SUPPORTED_DRONE_TYPES)
    parser.add_argument('--iris', help="What position to start irises in", choices=['0','1','2','3'], default=[], type=str, nargs="*")
    parser.add_argument('--plane', help="What position to start planes in", choices=['0','1','2','3'], default=[], type=str, nargs="*")
    parser.add_argument('--typhoon', help="What position to start typhoons in", choices=['0','1','2','3'], default=[], type=str, nargs="*")

    rclpy.init(args=sys.argv)

    # myargv = rospy_myargv(argv=sys.argv)[1:]
    # print("myargv", myargv)
    args = parser.parse_args(args=sys.argv[1:])

    overlap = set(args.iris) & set(args.plane)
    if overlap:
        parser.error("iris and plane poses cannot overlap %s" % overlap)
    overlap = set(args.iris) & set(args.typhoon)
    if overlap:
        parser.error("iris and typhoon poses cannot overlap %s" % overlap)
    overlap = set(args.typhoon) & set(args.plane)
    if overlap:
        parser.error("typhoon and plane poses cannot overlap %s" % overlap)

    drones = {}
    if not (args.iris + args.plane + args.typhoon):
        ds = DroneSelector()
        ds.mainloop()
        drones = ds.drones
        print("DRONES is", drones)

    for id in args.iris:
        drones[id] = Drone('iris', starting_poses[id], id)
    for id in args.plane:
        drones[id] = Drone('plane', starting_poses[id], id)
    for id in args.typhoon:
        drones[id] = Drone('typhoon_h480', starting_poses[id], id)
    run_drones(drones)

if __name__ == '__main__':
    main()
