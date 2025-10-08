#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Dobot Magician movement + tool test via roslibpy (ROSBridge)

Features:
- Connect to ROSBridge
- Publish joint trajectory and end-effector pose
- Get current joint state (subscriber cache + optional wait)
- Tool control: on+open, on+close, off (via /dobot_magician/target_tool_state Int32MultiArray [onOff, openClose])
- Optional safety commands (init / E-Stop) if your stack supports them

Run:
    python dobot_roslibpy_control.py
"""

import time
import math
import threading
import roslibpy


def quaternion_from_euler(roll, pitch, yaw):
    """RPY (rad) to quaternion (x, y, z, w) — ZYX convention."""
    cr = math.cos(roll * 0.5);  sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5);   sy = math.sin(yaw * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


def make_joint_traj_msg(positions, duration_sec=1.0, joint_names=None):
    """trajectory_msgs/JointTrajectory message as dict."""
    if joint_names is None:
        joint_names = []  # Fill if controller requires specific names
    secs = int(duration_sec)
    nsecs = int((duration_sec - secs) * 1e9)
    return {
        'joint_names': joint_names,
        'points': [{
            'positions': list(positions),
            'velocities': [],
            'accelerations': [],
            'effort': [],
            'time_from_start': {'secs': secs, 'nsecs': nsecs}
        }]
    }


def make_pose_msg(xyz, rpy):
    """geometry_msgs/Pose message as dict."""
    qx, qy, qz, qw = quaternion_from_euler(rpy[0], rpy[1], rpy[2])
    return {
        'position': {'x': float(xyz[0]), 'y': float(xyz[1]), 'z': float(xyz[2])},
        'orientation': {'x': qx, 'y': qy, 'z': qz, 'w': qw}
    }


class DobotClient:
    def __init__(self, host='192.168.27.1', port=9090):
        self.ros = roslibpy.Ros(host=host, port=port)
        self._joint_state_lock = threading.Lock()
        self._joint_state_event = threading.Event()
        self._latest_joint_state = None  # dict of sensor_msgs/JointState

        # Publishers
        self.pub_joint = roslibpy.Topic(self.ros,
                                        '/dobot_magician/target_joint_states',
                                        'trajectory_msgs/JointTrajectory')
        self.pub_pose = roslibpy.Topic(self.ros,
                                       '/dobot_magician/target_end_effector_pose',
                                       'geometry_msgs/Pose')
        self.pub_safety = roslibpy.Topic(self.ros,
                                         '/dobot_magician/target_safety_status',
                                         'std_msgs/Int32')
        self.pub_tool = roslibpy.Topic(self.ros,
                                       '/dobot_magician/target_tool_state',
                                       'std_msgs/Int32MultiArray')  # Data=[onOff, openClose]

        # Subscribers
        self.sub_joint_state = roslibpy.Topic(self.ros,
                                              '/dobot_magician/joint_states',
                                              'sensor_msgs/JointState')

    # ---------------- Connection ----------------
    def connect(self, timeout=5.0):
        self.ros.run()
        start = time.time()
        while not self.ros.is_connected and (time.time() - start) < timeout:
            time.sleep(0.05)
        if not self.ros.is_connected:
            raise RuntimeError('Failed to connect to ROSBridge')

        # Register pubs/subs
        self.pub_joint.advertise()
        self.pub_pose.advertise()
        self.pub_safety.advertise()
        self.pub_tool.advertise()

        self.sub_joint_state.subscribe(self._on_joint_state)

    def close(self):
        try:
            self.sub_joint_state.unsubscribe()
        except Exception:
            pass
        for t in (self.pub_joint, self.pub_pose, self.pub_safety, self.pub_tool):
            try:
                t.unadvertise()
            except Exception:
                pass
        self.ros.terminate()

    # ---------------- Callbacks ----------------
    def _on_joint_state(self, msg):
        with self._joint_state_lock:
            self._latest_joint_state = msg
            self._joint_state_event.set()

    # ---------------- High-level API ----------------
    def move_joints(self, positions, duration_sec=1.0, joint_names=None):
        self.pub_joint.publish(roslibpy.Message(make_joint_traj_msg(positions, duration_sec, joint_names)))

    def move_ee_pose(self, xyz, rpy):
        self.pub_pose.publish(roslibpy.Message(make_pose_msg(xyz, rpy)))

    def estop(self):
        self.pub_safety.publish(roslibpy.Message({'data': 3}))  # 3 = ESTOP

    def initialise(self):
        self.pub_safety.publish(roslibpy.Message({'data': 2}))  # 2 = INITIALISATION

    # ---------------- Joint state helpers ----------------
    def get_joint_state(self, wait_timeout: float = 1.0):
        """
        Returns latest JointState as a dict:
          {'name': [...], 'position': [...], 'velocity': [...], 'effort': [...]}
        If no message yet, waits up to wait_timeout seconds; returns {} if none.
        """
        if not self._latest_joint_state:
            self._joint_state_event.clear()
            self._joint_state_event.wait(timeout=wait_timeout)
        with self._joint_state_lock:
            return dict(self._latest_joint_state) if self._latest_joint_state else {}

    def get_joint_positions(self, wait_timeout: float = 1.0):
        js = self.get_joint_state(wait_timeout)
        return js.get('position', []) if js else []

    # ---------------- Tool control ----------------
    def tool_on_open(self):
        """OnOff=1, OpenClose=1 (open gripper / vacuum off to release depending on tool)."""
        self.pub_tool.publish(roslibpy.Message({'data': [1, 1]}))

    def tool_on_close(self):
        """OnOff=1, OpenClose=0 (close gripper / vacuum on to pick depending on tool)."""
        self.pub_tool.publish(roslibpy.Message({'data': [1, 0]}))

    def tool_off(self):
        """OnOff=0, OpenClose=0 (tool off)."""
        self.pub_tool.publish(roslibpy.Message({'data': [0, 0]}))


def demo_sequence():
    bot = DobotClient(host='192.168.27.1', port=9090)
    bot.connect()

    try:
        # Basic joint move
        bot.move_joints([0.0, 0.4, 0.3, 0.0], duration_sec=1.0)
        time.sleep(0.8)

        # Basic EE pose move
        bot.move_ee_pose([0.20, 0.00, 0.10], [0.0, 0.0, 0.0])
        time.sleep(0.8)

        # Tool tests
        bot.tool_on_open()
        time.sleep(5)
        bot.tool_on_close()
        time.sleep(5)
        bot.tool_off()
        time.sleep(1)

        # Sweep second joint
        for k in range(0, 19):  # 0.1:0.05:1.0
            j2 = round(0.1 + 0.05 * k, 3)
            bot.move_joints([0.0, j2, 0.3, 0.0], duration_sec=0.2)
            time.sleep(0.1)

        # Read current joints
        joints = bot.get_joint_positions(wait_timeout=1.0)
        print('Current joints:', joints)

        # Optional safety pulse
        bot.estop()
        time.sleep(0.5)
        bot.initialise()
        time.sleep(1.0)

        # Final confirmation move
        bot.move_joints([0.0, 0.3, 0.2, 0.0], duration_sec=0.8)
        time.sleep(0.8)

        # Fetch a full joint state
        js = bot.get_joint_state(wait_timeout=1.0)
        print('Full joint state:', js)

    finally:
        bot.close()


if __name__ == '__main__':
    demo_sequence()
