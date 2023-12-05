#!/usr/bin/env python

from __future__ import absolute_import, division, print_function

from threading import Lock

import numpy as np
import rospy
import tf2_ros
from mushr_base import utils
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped

"""
Publishes joint and tf information about the racecar
"""


class RacecarState:
    """
    __init__: Initialize params, publishers, subscribers, and timer
    """

    def __init__(self):
        # speed (rpm) = self.SPEED_TO_ERPM_OFFSET + self.SPEED_TO_ERPM_GAIN * speed (m/s)
        self.SPEED_TO_ERPM_OFFSET = float(
            rospy.get_param("vesc/speed_to_erpm_offset", 0.0)
        )
        self.SPEED_TO_ERPM_GAIN = float(
            rospy.get_param("vesc/speed_to_erpm_gain", 4614.0)
        )

        # servo angle = self.STEERING_TO_SERVO_OFFSET + self.STEERING_TO_SERVO_GAIN * steering_angle (rad)
        self.STEERING_TO_SERVO_OFFSET = float(
            rospy.get_param("vesc/steering_angle_to_servo_offset", 0.5304)
        )
        self.STEERING_TO_SERVO_GAIN = float(
            rospy.get_param("vesc/steering_angle_to_servo_gain", -1.2135)
        )

        # Length of the car
        self.CAR_LENGTH = float(rospy.get_param("vesc/chassis_length", 0.33))

        # Width of the car
        self.CAR_WIDTH = float(rospy.get_param("vesc/wheelbase", 0.25))

        # The radius of the car wheel in meters
        self.CAR_WHEEL_RADIUS = 0.0976 / 2.0

        # Rate at which to publish joints and tf
        self.UPDATE_RATE = float(rospy.get_param("~update_rate", 20.0))

        # Speed noise mean is computed as the most recent speed multiplied by this value
        self.SPEED_OFFSET = float(rospy.get_param("~speed_offset", 0.00))

        # Speed noise std dev
        self.SPEED_NOISE = float(rospy.get_param("~speed_noise", 0.0001))

        # Steering angle noise mean is cimputed as the most recent steering angle multiplied by this value
        self.STEERING_ANGLE_OFFSET = float(
            rospy.get_param("~steering_angle_offset", 0.00)
        )

        # Steering angle noise std dev
        self.STEERING_ANGLE_NOISE = float(
            rospy.get_param("~steering_angle_noise", 0.000001)
        )

        # Forward direction noise mean
        self.FORWARD_OFFSET = float(rospy.get_param("~forward_offset", 0.0))

        # Forward direction noise std dev
        self.FORWARD_FIX_NOISE = float(rospy.get_param("~forward_fix_noise", 0.0000001))

        # Additional zero-mean gaussian noise added to forward direction
        # std dev is most recent velocity times this value
        self.FORWARD_SCALE_NOISE = float(rospy.get_param("~forward_scale_noise", 0.001))

        # Side direction noise mean
        self.SIDE_OFFSET = float(rospy.get_param("~side_offset", 0.0))

        # Side direction noise std dev
        self.SIDE_FIX_NOISE = float(rospy.get_param("~side_fix_noise", 0.000001))

        # Additional zero-mean gaussian noise added to side direction
        # std dev is most recent velocity times this value
        self.SIDE_SCALE_NOISE = float(rospy.get_param("~side_scale_noise", 0.001))

        # Theta noise mean
        self.THETA_OFFSET = float(rospy.get_param("~theta_offset", 0.0))

        # Theta noise std dev
        self.THETA_FIX_NOISE = float(rospy.get_param("~theta_fix_noise", 0.000001))

        # Append this prefix to any broadcasted TFs
        self.TF_PREFIX = str(rospy.get_param("~tf_prefix", "").rstrip("/"))
        if len(self.TF_PREFIX) > 0:
            self.TF_PREFIX = self.TF_PREFIX + "/"

        # The most recent time stamp
        self.last_stamp = None

        # The most recent speed (m/s)
        self.last_speed = 0.0
        self.last_speed_lock = Lock()

        # The most recent steering angle (rad)
        self.last_steering_angle = 0.0
        self.last_steering_angle_lock = Lock()

        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=1)

        # The most recent transform from odom to base_footprint
        self.cur_odom_to_base_trans = np.array([0, 0], dtype=np.float)
        self.cur_odom_to_base_rot = 0.0
        self.cur_odom_to_base_lock = Lock()

        # Message used to publish joint values
        self.joint_msg = JointState()
        self.joint_msg.name = [
            "front_left_wheel_throttle",
            "front_right_wheel_throttle",
            "back_left_wheel_throttle",
            "back_right_wheel_throttle",
            "front_left_wheel_steer",
            "front_right_wheel_steer",
        ]
        self.joint_msg.position = [0, 0, 0, 0, 0, 0]
        self.joint_msg.velocity = []
        self.joint_msg.effort = []

        # Publishes joint messages
        self.br = tf2_ros.TransformBroadcaster()

        self.tf_buffer = tf2_ros.Buffer()
        # Duration param controls how often to publish default map to odom tf
        # if no other nodes are publishing it
        self.transformer = tf2_ros.TransformListener(self.tf_buffer)

        # Publishes joint values
        self.cur_joints_pub = rospy.Publisher("joint_states", JointState, queue_size=1)

        # Subscribes to info about the bldc (particularly the speed in rpm)
        self.speed_sub = rospy.Subscriber(
            "vesc/sensors/core", VescStateStamped, self.speed_cb, queue_size=1
        )

        # Subscribes to the position of the servo arm
        self.servo_sub = rospy.Subscriber(
            "vesc/sensors/servo_position_command", Float64, self.servo_cb, queue_size=1
        )

        # Timer to updates joints and tf
        self.update_timer = rospy.Timer(
            rospy.Duration.from_sec(1.0 / self.UPDATE_RATE), self.timer_cb
        )

    """
    clip_angle: Clip an angle to be between -pi and pi
      val: Angle in radians
      Returns: Equivalent angle between -pi and pi (rad)
    """

    def clip_angle(self, val):
        while val > np.pi:
            val -= 2 * np.pi

        while val < -np.pi:
            val += 2 * np.pi
        return val

    """
    speed_cb: Callback to capture the speed of the car
      msg: vesc_msgs/VescStateStamped message containing the speed of the car (rpm)
    """

    def speed_cb(self, msg):
        self.last_speed_lock.acquire()
        self.last_speed = (
                                  msg.state.speed - self.SPEED_TO_ERPM_OFFSET
                          ) / self.SPEED_TO_ERPM_GAIN
        self.last_speed_lock.release()

    """
    servo_cb: Callback to capture the steering angle of the car
      msg: std_msgs/Float64 message containing the servo value
    """

    def servo_cb(self, msg):
        self.last_steering_angle_lock.acquire()
        self.last_steering_angle = (
                                           msg.data - self.STEERING_TO_SERVO_OFFSET
                                   ) / self.STEERING_TO_SERVO_GAIN
        self.last_steering_angle_lock.release()

    """
    timer_cb: Callback occurring at a rate of self.UPDATE_RATE. Updates the car joint
              angles and tf of the base_footprint w.r.t odom. Also publishes robot state 
              as a PoseStamped msg.
      event: Information about when this callback occurred
    """

    def timer_cb(self, event):
        now = rospy.Time.now()

        # Get the time since the last update
        if self.last_stamp is None:
            self.last_stamp = now
        dt = (now - self.last_stamp).to_sec()

        # Add noise to the speed
        self.last_speed_lock.acquire()
        v = self.last_speed + np.random.normal(
            loc=self.SPEED_OFFSET * self.last_speed, scale=self.SPEED_NOISE, size=1
        )
        self.last_speed_lock.release()

        # Add noise to the steering angle
        self.last_steering_angle_lock.acquire()
        delta = self.last_steering_angle + np.random.normal(
            loc=self.STEERING_ANGLE_OFFSET * self.last_steering_angle,
            scale=self.STEERING_ANGLE_NOISE,
            size=1,
        )
        self.last_steering_angle_lock.release()

        self.cur_odom_to_base_lock.acquire()

        # Apply kinematic car model to the previous pose
        new_pose = np.array(
            [
                self.cur_odom_to_base_trans[0],
                self.cur_odom_to_base_trans[1],
                self.cur_odom_to_base_rot,
            ],
            dtype=np.float,
        )
        if np.abs(delta) < 1e-2:
            # Changes in x, y, and theta
            dtheta = 0
            dx = v * np.cos(self.cur_odom_to_base_rot) * dt
            dy = v * np.sin(self.cur_odom_to_base_rot) * dt

            # New joint values
            joint_left_throttle = v * dt / self.CAR_WHEEL_RADIUS
            joint_right_throttle = v * dt / self.CAR_WHEEL_RADIUS
            joint_left_steer = 0.0
            joint_right_steer = 0.0

        else:
            # Changes in x, y, and theta
            tan_delta = np.tan(delta)
            dtheta = ((v / self.CAR_LENGTH) * tan_delta) * dt
            dx = (self.CAR_LENGTH / tan_delta) * (
                    np.sin(self.cur_odom_to_base_rot + dtheta)
                    - np.sin(self.cur_odom_to_base_rot)
            )
            dy = (self.CAR_LENGTH / tan_delta) * (
                    -1 * np.cos(self.cur_odom_to_base_rot + dtheta)
                    + np.cos(self.cur_odom_to_base_rot)
            )

            # New joint values
            # Apply kinematic car model to compute wheel deltas
            h_val = (self.CAR_LENGTH / tan_delta) - (self.CAR_WIDTH / 2.0)
            joint_outer_throttle = (
                    ((self.CAR_WIDTH + h_val) / (0.5 * self.CAR_WIDTH + h_val))
                    * v
                    * dt
                    / self.CAR_WHEEL_RADIUS
            )
            joint_inner_throttle = (
                    ((h_val) / (0.5 * self.CAR_WIDTH + h_val))
                    * v
                    * dt
                    / self.CAR_WHEEL_RADIUS
            )
            joint_outer_steer = np.arctan2(self.CAR_LENGTH, self.CAR_WIDTH + h_val)
            joint_inner_steer = np.arctan2(self.CAR_LENGTH, h_val)

            # Assign joint values according to whether we are turning left or right
            if (delta) > 0.0:
                joint_left_throttle = joint_inner_throttle
                joint_right_throttle = joint_outer_throttle
                joint_left_steer = joint_inner_steer
                joint_right_steer = joint_outer_steer

            else:
                joint_left_throttle = joint_outer_throttle
                joint_right_throttle = joint_inner_throttle
                joint_left_steer = joint_outer_steer - np.pi
                joint_right_steer = joint_inner_steer - np.pi

        # Apply kinematic model updates and noise to the new pose
        new_pose[0] += (
                dx
                + np.random.normal(
            loc=self.FORWARD_OFFSET, scale=self.FORWARD_FIX_NOISE, size=1
        )
                + np.random.normal(
            loc=0.0, scale=np.abs(v) * self.FORWARD_SCALE_NOISE, size=1
        )
        )
        new_pose[1] += (
                dy
                + np.random.normal(loc=self.SIDE_OFFSET, scale=self.SIDE_FIX_NOISE, size=1)
                + np.random.normal(loc=0.0, scale=np.abs(v) * self.SIDE_SCALE_NOISE, size=1)
        )
        new_pose[2] += dtheta + np.random.normal(
            loc=self.THETA_OFFSET, scale=self.THETA_FIX_NOISE, size=1
        )

        new_pose[2] = self.clip_angle(new_pose[2])

        # Update pose of base_footprint w.r.t odom
        self.cur_odom_to_base_trans[0] = new_pose[0]
        self.cur_odom_to_base_trans[1] = new_pose[1]
        self.cur_odom_to_base_rot = new_pose[2]

        # Update joint values
        self.joint_msg.position[0] += joint_left_throttle
        self.joint_msg.position[1] += joint_right_throttle
        self.joint_msg.position[2] += joint_left_throttle
        self.joint_msg.position[3] += joint_right_throttle
        self.joint_msg.position[4] = joint_left_steer
        self.joint_msg.position[5] = joint_right_steer

        # Clip all joint angles
        for i in range(len(self.joint_msg.position)):
            self.joint_msg.position[i] = self.clip_angle(self.joint_msg.position[i])

        t = utils.make_transform_msg(self.cur_odom_to_base_trans, self.cur_odom_to_base_rot,
                                     self.TF_PREFIX + "base_footprint", self.TF_PREFIX + "odom")

        # Publish the tf from odom to base_footprint
        self.br.sendTransform(t)

        # Publish the joint states
        self.joint_msg.header.stamp = now
        self.cur_joints_pub.publish(self.joint_msg)

        self.last_stamp = now

        odom_msg = Odometry()
        odom_msg.header.stamp = self.last_stamp
        odom_msg.header.frame_id = self.TF_PREFIX + "odom"
        odom_msg.pose.pose.position = t.transform.translation
        odom_msg.pose.pose.orientation = t.transform.rotation

        odom_msg.child_frame_id = self.TF_PREFIX + "base_link"
        odom_msg.twist.twist.linear.x = dx
        odom_msg.twist.twist.linear.y = dy
        odom_msg.twist.twist.angular.z = dtheta

        self.odom_pub.publish(odom_msg)
        self.cur_odom_to_base_lock.release()
