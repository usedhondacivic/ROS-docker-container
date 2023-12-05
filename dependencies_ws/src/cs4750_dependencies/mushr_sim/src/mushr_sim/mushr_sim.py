#!/usr/bin/env python

from __future__ import absolute_import, division, print_function

from threading import Lock

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from mushr_base import utils
from mushr_sim.fake_urg import FakeURG
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetMap
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped
from mushr_sim.srv import CarPose

"""
Publishes joint and tf information about the racecar
"""


class MushrSim:
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

        # Disable publishing car_pose if true.
        self.USE_MOCAP = bool(rospy.get_param("~use_mocap", False))

        initial_x = float(rospy.get_param("~initial_x", 0.0))
        initial_y = float(rospy.get_param("~initial_y", 0.0))
        initial_z = float(rospy.get_param("~initial_z", 0.0))
        initial_theta = float(rospy.get_param("~initial_theta", 0.0))

        # Append this prefix to any broadcasted TFs
        self.TF_PREFIX = str(rospy.get_param("~tf_prefix", "").rstrip("/"))
        if len(self.TF_PREFIX) > 0:
            self.TF_PREFIX = self.TF_PREFIX + "/"

        # The map and map params
        self.permissible_region = None
        self.map_info = None

        # Get the map
        self.permissible_region, self.map_info, raw_map_msg = self.get_map()

        # The most recent time stamp
        self.last_stamp = None

        # The most recent speed (m/s)
        self.last_speed = 0.0
        self.last_speed_lock = Lock()

        # The most recent steering angle (rad)
        self.last_steering_angle = 0.0
        self.last_steering_angle_lock = Lock()

        # The most recent transform from odom to base_footprint
        self.cur_odom_to_base_trans = np.array([initial_x, initial_y], dtype=np.float)
        self.cur_odom_to_base_rot = initial_theta
        self.cur_odom_to_base_lock = Lock()

        # Internal transform from the map to odom
        self.cur_map_to_odom_trans = np.array([0, 0], dtype=np.float)
        self.cur_map_to_odom_rot = 0
        self.cur_map_to_odom_lock = Lock()

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

        CAR_NAME = "car"

        self.fake_laser = FakeURG(raw_map_msg, topic_namespace="car/")
        # Publishes joint values
        if not self.USE_MOCAP:
            self.state_pub = rospy.Publisher("~{}/car_pose".format(CAR_NAME), PoseStamped, queue_size=1)

        self.odom_pub = rospy.Publisher("~{}/odom".format(CAR_NAME), Odometry, queue_size=1)

        # Publishes joint values
        self.cur_joints_pub = rospy.Publisher("~{}/joint_states".format(CAR_NAME), JointState, queue_size=1)

        # Subscribes to the initial pose of the car
        self.init_pose_sub = rospy.Subscriber("~reposition", PoseStamped, self.init_pose_cb, queue_size=1)

        # Subscribes to info about the bldc (particularly the speed in rpm)
        self.speed_sub = rospy.Subscriber(
            "/car/vesc/sensors/core", VescStateStamped, self.speed_cb, queue_size=1
        )

        # Subscribes to the position of the servo arm
        self.servo_sub = rospy.Subscriber(
            "/car/vesc/sensors/servo_position_command", Float64, self.servo_cb, queue_size=1
        )

        # Timer to updates joints and tf
        self.update_timer = rospy.Timer(
            rospy.Duration.from_sec(1.0 / self.UPDATE_RATE), self.timer_cb
        )

        self._car_reposition_srv = rospy.Service("~reposition", CarPose, self._car_reposition_cb)

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
    init_pose_cb: Callback to capture the initial pose of the car
      msg: geometry_msg/PoseStamped containing the initial pose
    """

    def init_pose_cb(self, msg):
        # Get the pose of the car w.r.t the map in meters
        rx_trans = np.array(
            [msg.pose.position.x, msg.pose.position.y], dtype=np.float
        )
        rx_rot = utils.quaternion_to_angle(msg.pose.orientation)

        # Get the pose of the car w.r.t the map in pixels
        if self.map_info is not None:
            map_rx_pose = utils.world_to_map(
                (rx_trans[0], rx_trans[1], rx_rot), self.map_info
            )
            # Update the pose of the car if either bounds checking is not enabled,
            # or bounds checking is enabled but the car is in-bounds
            if not self._check_position_in_bounds(map_rx_pose[0], map_rx_pose[1]):
                rospy.logwarn("Requested reposition into obstacle. Ignoring.")
                return

        with self.cur_odom_to_base_lock:
            # Move the vehicle by updating the odom->base transform
            self.cur_odom_to_base_trans = rx_trans
            self.cur_odom_to_base_rot = rx_rot

    def _check_position_in_bounds(self, x, y):
        if self.permissible_region is None:
            return True
        return (
                0 <= x < self.permissible_region.shape[1] and \
                0 <= y < self.permissible_region.shape[0] and \
                self.permissible_region[
                    int(y + 0.5), int(x + 0.5)
                ]
        )

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
              angles and tf of the base_footprint w.r.t odom. Will also publish
              the tf between odom and map if it detects that no such tf is already
              being published. Also publishes robot state as a PoseStamped msg.
      event: Information about when this callback occurred
    """

    def timer_cb(self, event):
        now = rospy.Time.now()

        # Get the time since the last update
        if self.last_stamp is None:
            self.last_stamp = now
        dt = (now - self.last_stamp).to_sec()

        # Add noise to the speed
        with self.last_speed_lock:
            v = self.last_speed + np.random.normal(
                loc=self.SPEED_OFFSET * self.last_speed, scale=self.SPEED_NOISE, size=1
            )


        # Add noise to the steering angle
        with self.last_steering_angle_lock:
            delta = self.last_steering_angle + np.random.normal(
                loc=self.STEERING_ANGLE_OFFSET * self.last_steering_angle,
                scale=self.STEERING_ANGLE_NOISE,
                size=1,
            )

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
            # Applt kinematic car model to compute wheel deltas
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

        in_bounds = True
        if self.permissible_region is not None:
            # Compute the new pose w.r.t the map in meters
            new_map_pose = np.zeros(3, dtype=np.float)
            new_map_pose[0] = self.cur_map_to_odom_trans[0] + (
                    new_pose[0] * np.cos(self.cur_map_to_odom_rot)
                    - new_pose[1] * np.sin(self.cur_map_to_odom_rot)
            )
            new_map_pose[1] = self.cur_map_to_odom_trans[1] + (
                    new_pose[0] * np.sin(self.cur_map_to_odom_rot)
                    + new_pose[1] * np.cos(self.cur_map_to_odom_rot)
            )
            new_map_pose[2] = self.cur_map_to_odom_rot + new_pose[2]

            # Get the new pose w.r.t the map in pixels
            if self.map_info is not None:
                new_map_pose = utils.world_to_map(new_map_pose, self.map_info)
                in_bounds = self._check_position_in_bounds(new_map_pose[0], new_map_pose[1])

        if in_bounds:
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
        else:
            rospy.logwarn_throttle(1, "Not in bounds")

        t = utils.make_transform_msg(self.cur_odom_to_base_trans, self.cur_odom_to_base_rot,
                                     self.TF_PREFIX + "ground_truth_base_footprint", self.TF_PREFIX + "map")

        # Tell the laser where we are
        # rospy.logerr_throttle(1,t)
        # rospy.logerr_throttle(1, new_pose)
        self.fake_laser.transform = t.transform
        # Publish the tf from odom to base_footprint
        self.br.sendTransform(t)

        # Publish the joint states
        self.joint_msg.header.stamp = now
        self.cur_joints_pub.publish(self.joint_msg)

        self.last_stamp = now

        self.cur_odom_to_base_lock.release()

        # Publish current state as a PoseStamped topic
        if not self.USE_MOCAP:
            cur_pose = PoseStamped()
            cur_pose.header.frame_id = "map"
            cur_pose.header.stamp = now
            cur_pose.pose.position.x = (
                    self.cur_odom_to_base_trans[0] + self.cur_map_to_odom_trans[0]
            )
            cur_pose.pose.position.y = (
                    self.cur_odom_to_base_trans[1] + self.cur_map_to_odom_trans[1]
            )
            cur_pose.pose.position.z = 0.0
            rot = self.cur_odom_to_base_rot + self.cur_map_to_odom_rot
            cur_pose.pose.orientation = utils.angle_to_quaternion(rot)
            self.state_pub.publish(cur_pose)

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

    def _car_reposition_cb(self, request):
        # Get the pose of the car w.r.t the map in meters
        rx_trans = np.array(
            [request.x, request.y], dtype=np.float
        )
        rx_rot = request.theta

        # Get the pose of the car w.r.t the map in pixels
        if self.map_info is not None:
            map_rx_pose = utils.world_to_map(
                (rx_trans[0], rx_trans[1], rx_rot), self.map_info
            )
            # Update the pose of the car if either bounds checking is not enabled,
            # or bounds checking is enabled but the car is in-bounds
            if not self._check_position_in_bounds(map_rx_pose[0], map_rx_pose[1]):
                rospy.logwarn("Requested reposition into obstacle. Ignoring.")
                return

        with self.cur_odom_to_base_lock:
            # Move the vehicle by updating the odom->base transform
            self.cur_odom_to_base_trans = rx_trans
            self.cur_odom_to_base_rot = rx_rot

        with self.cur_odom_to_base_lock:
            self.cur_odom_to_base_trans = np.array([request.x, request.y], dtype=np.float)
            self.cur_odom_to_base_rot = request.theta
        return True

    """
    get_map: Get the map and map meta data
      Returns: A tuple
                First element is array representing map
                  0 indicates out of bounds, 1 indicates in bounds
                Second element is nav_msgs/MapMetaData message with meta data about the map
    """

    def get_map(self):
        # Use the 'static_map' service (launched by MapServer.launch) to get the map
        map_service_name = rospy.get_param("~static_map", "static_map")
        rospy.wait_for_service(map_service_name)
        map_msg = rospy.ServiceProxy(map_service_name, GetMap)().map
        map_info = map_msg.info  # Save info about map for later use

        # Create numpy array representing map for later use
        array_255 = np.array(map_msg.data).reshape(
            (map_msg.info.height, map_msg.info.width)
        )
        permissible_region = np.zeros_like(array_255, dtype=bool)
        permissible_region[
            array_255 == 0
            ] = 1  # Numpy array of dimension (map_msg.info.height, map_msg.info.width),
        # With values 0: not permissible, 1: permissible
        return permissible_region, map_info, map_msg
