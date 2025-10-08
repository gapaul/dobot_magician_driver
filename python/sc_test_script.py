#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Dobot Magician movement + tool test via rospy (ROS Noetic)

Features:
- Connect with rospy
- Publish joint trajectory and end-effector pose
- Get current joint state (subscriber cache + optional wait)
- Tool control: on+open, on+close, off (via /dobot_magician/target_tool_state Int32MultiArray [onOff, openClose])
- Safety commands (init / E-Stop) if your stack supports them

Run:
    rosrun <your_pkg> dobot_rospy_control.py
"""

import math
import threading
import time
from typing import List, Optional, Dict

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32, Int32MultiArray
from tf.transformations import quaternion_from_euler


def make_joint_traj_msg(positions: List[float], duration_sec: float = 1.0, joint_names: Optional[List[str]] = None):
    if joint_names is None:
        joint_names = []  # fill if your controller requires specific names
    pt = JointTrajectoryPoint()
    pt.positions = list(positions)
    pt.time_from_start = rospy.Duration.from_sec(duration_sec)

    jt = JointTrajectory()
    jt.joint_names = joint_names
    jt.points = [pt]
    return jt


def make_pose_msg(xyz, rpy):
    qx, qy, qz, qw = quaternion_from_euler(rpy[0], rpy[1], rpy[2])
    pose = Pose()
    pose.position = Point(float(xyz[0]), float(xyz[1]), float(xyz[2]))
    pose.orientation = Quaternion(qx, qy, qz, qw)
    return pose


class DobotNode:
    def __init__(self, ns: str = "/dobot_magician"):
        self.ns = ns

        # Publishers
        self.pub_joint = rospy.Publisher(f"{ns}/target_joint_states", JointTrajectory, queue_size=1)
        self.pub_pose = rospy.Publisher(f"{ns}/target_end_effector_pose", Pose, queue_size=1)
        self.pub_safety = rospy.Publisher(f"{ns}/target_safety_status", Int32, queue_size=1)
        self.pub_tool = rospy.Publisher(f"{ns}/target_tool_state", Int32MultiArray, queue_size=1)

        # Subscribers
        self._joint_state_lock = threading.Lock()
        self._joint_state_event = threading.Event()
        self._latest_joint_state: Optional[JointState] = None

        self.sub_joint = rospy.Subscriber(f"{ns}/joint_states", JointState, self._on_joint_state, queue_size=1)

        # Allow pubs to register
        rospy.sleep(0.3)

    def _on_joint_state(self, msg: JointState):
        with self._joint_state_lock:
            self._latest_joint_state = msg
            self._joint_state_event.set()

    # -------- Movement --------
    def move_joints(self, positions: List[float], duration_sec: float = 1.0, joint_names: Optional[List[str]] = None):
        self.pub_joint.publish(make_joint_traj_msg(positions, duration_sec, joint_names))

    def move_ee_pose(self, xyz, rpy):
        self.pub_pose.publish(make_pose_msg(xyz, rpy))

    # -------- Safety --------
    def estop(self):
        self.pub_safety.publish(Int32(data=3))  # 3 = ESTOP

    def initialise(self):
        self.pub_safety.publish(Int32(data=2))  # 2 = INITIALISATION

    # -------- Joint state --------
    def get_joint_state(self, wait_timeout: float = 1.0) -> Dict:
        """
        Returns dict with keys: name, position, velocity, effort.
        Waits up to wait_timeout seconds if nothing received yet.
        """
        if self._latest_joint_state is None:
            self._joint_state_event.clear()
            self._joint_state_event.wait(timeout=wait_timeout)
        with self._joint_state_lock:
            if self._latest_joint_state is None:
                try:
                    msg = rospy.wait_for_message(f"{self.ns}/joint_states", JointState, timeout=wait_timeout)
                    self._latest_joint_state = msg
                except rospy.ROSException:
                    return {}
            msg = self._latest_joint_state
            return {
                "name": list(msg.name),
                "position": list(msg.position),
                "velocity": list(msg.velocity),
                "effort": list(msg.effort),
            }

    def get_joint_positions(self, wait_timeout: float = 1.0) -> List[float]:
        js = self.get_joint_state(wait_timeout)
        return js.get("position", []) if js else []

    # -------- Tool control --------
    def tool_on_open(self):
        self.pub_tool.publish(Int32MultiArray(data=[1, 1]))

    def tool_on_close(self):
        self.pub_tool.publish(Int32MultiArray(data=[1, 0]))

    def tool_off(self):
        self.pub_tool.publish(Int32MultiArray(data=[0, 0]))


def demo_sequence():
    rospy.loginfo("Starting Dobot Magician demo sequence (rospy).")
    bot = DobotNode(ns="/dobot_magician")

    # Basic joint move
    bot.move_joints([0.0, 0.4, 0.3, 0.0], duration_sec=1.0)
    rospy.sleep(0.8)

    # Basic EE pose move
    bot.move_ee_pose([0.20, 0.00, 0.10], [0.0, 0.0, 0.0])
    rospy.sleep(0.8)

    # Tool tests
    bot.tool_on_open()
    rospy.sleep(0.5)
    bot.tool_on_close()
    rospy.sleep(0.5)
    bot.tool_off()
    rospy.sleep(0.3)

    # Sweep second joint
    for k in range(0, 19):  # 0.1:0.05:1.0
        j2 = round(0.1 + 0.05 * k, 3)
        bot.move_joints([0.0, j2, 0.3, 0.0], duration_sec=0.2)
        rospy.sleep(0.1)

    # Read current joints
    joints = bot.get_joint_positions(wait_timeout=1.0)
    rospy.loginfo(f"Current joints: {joints}")

    # Optional safety pulse
    bot.estop()
    rospy.sleep(0.5)
    bot.initialise()
    rospy.sleep(1.0)

    # Final confirmation move
    bot.move_joints([0.0, 0.3, 0.2, 0.0], duration_sec=0.8)
    rospy.sleep(0.8)

    # Fetch full joint state
    js = bot.get_joint_state(wait_timeout=1.0)
    rospy.loginfo(f"Full joint state: {js}")


if __name__ == "__main__":
    rospy.init_node("dobot_rospy_control", anonymous=True)
    try:
        demo_sequence()
    finally:
        # Give publishers a moment to flush before shutdown
        time.sleep(0.2)
