#!/usr/bin/env python

# Copyright (c) 2019, The Personal Robotics Lab, The MuSHR Team, The Contributors of MuSHR
# License: BSD 3-Clause. See LICENSE.md file in root directory.

from __future__ import absolute_import, division, print_function

import numpy as np
import range_libc
import rospy
import tf2_ros
from geometry_msgs.msg import Quaternion
from nav_msgs.srv import GetMap
from geometry_msgs.msg import Transform
from sensor_msgs.msg import LaserScan

from mushr_sim import utils


class FakeURG:
    def __init__(self, map_msg, topic_namespace=""):

        self.UPDATE_RATE = float(rospy.get_param("~update_rate", 10.0))
        self.THETA_DISCRETIZATION = float(rospy.get_param("~theta_discretization", 656))
        self.MIN_RANGE_METERS = float(rospy.get_param("~min_range_meters", 0.02))
        self.MAX_RANGE_METERS = float(rospy.get_param("~max_range_meters", 5.6))
        self.ANGLE_STEP = float(rospy.get_param("~angle_step", 0.00613592332229))
        self.ANGLE_MIN = float(rospy.get_param("~angle_min", -2.08621382713))
        self.ANGLE_MAX = float(rospy.get_param("~angle_max", 2.09234976768))
        self.ANGLES = np.arange(
            self.ANGLE_MIN, self.ANGLE_MAX, self.ANGLE_STEP, dtype=np.float32
        )
        self.CAR_LENGTH = float(rospy.get_param("~car_length", 0.33))
        self.Z_SHORT = float(rospy.get_param("~z_short", 0.03))
        self.Z_MAX = float(rospy.get_param("~z_max", 0.16))
        self.Z_BLACKOUT_MAX = float(rospy.get_param("~z_blackout_max", 50))
        self.Z_RAND = float(rospy.get_param("~z_rand", 0.01))
        self.Z_HIT = float(rospy.get_param("~z_hit", 0.8))
        self.Z_SIGMA = float(rospy.get_param("~z_sigma", 0.03))
        self.TF_PREFIX = str(rospy.get_param("~tf_prefix", "").rstrip("/"))
        if len(self.TF_PREFIX) > 0:
            self.TF_PREFIX = self.TF_PREFIX + "/"

        occ_map = range_libc.PyOMap(map_msg)
        max_range_px = int(self.MAX_RANGE_METERS / map_msg.info.resolution)
        self.range_method = range_libc.PyCDDTCast(
            occ_map, max_range_px, self.THETA_DISCRETIZATION
        )

        self._tf_buffer = tf2_ros.Buffer()
        self.tl = tf2_ros.TransformListener(self._tf_buffer)

        rate = rospy.Rate(10.0)
        # It almost always takes one cycle before the transforms arrive, so we'll wait
        # a bit by default
        rate.sleep()
        while not rospy.is_shutdown():
            try:
                transform = self._tf_buffer.lookup_transform(
                    self.TF_PREFIX + "base_link", self.TF_PREFIX + "laser_link", rospy.Time(0)
                )
                # Drop stamp header
                transform = transform.transform
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logwarn_throttle(5, e)
                rate.sleep()
                continue

        self.x_offset = transform.translation.x

        self.laser_pub = rospy.Publisher("~{}scan".format(topic_namespace), LaserScan, queue_size=1)

        self.update_timer = rospy.Timer(
            rospy.Duration.from_sec(1.0 / self.UPDATE_RATE), self.timer_cb
        )
        # Start at 0,0,0
        self.transform = Transform()
        self.transform.rotation.w = 1

    def noise_laser_scan(self, ranges):
        indices = np.zeros(ranges.shape[0], dtype=np.int)
        prob_sum = self.Z_HIT + self.Z_RAND + self.Z_SHORT
        hit_count = int((self.Z_HIT / prob_sum) * indices.shape[0])
        rand_count = int((self.Z_RAND / prob_sum) * indices.shape[0])
        short_count = indices.shape[0] - hit_count - rand_count
        indices[hit_count : hit_count + rand_count] = 1
        indices[hit_count + rand_count :] = 2
        np.random.shuffle(indices)

        hit_indices = indices == 0
        ranges[hit_indices] += np.random.normal(
            loc=0.0, scale=self.Z_SIGMA, size=hit_count
        )[:]

        rand_indices = indices == 1
        ranges[rand_indices] = np.random.uniform(
            low=self.MIN_RANGE_METERS, high=self.MAX_RANGE_METERS, size=rand_count
        )[:]

        short_indices = indices == 2
        ranges[short_indices] = np.random.uniform(
            low=self.MIN_RANGE_METERS, high=ranges[short_indices], size=short_count
        )[:]

        max_count = (self.Z_MAX / (prob_sum + self.Z_MAX)) * ranges.shape[0]
        while max_count > 0:
            cur = np.random.randint(low=0, high=ranges.shape[0], size=1)
            blackout_count = np.random.randint(low=1, high=self.Z_BLACKOUT_MAX, size=1)
            while (
                cur > 0
                and cur < ranges.shape[0]
                and blackout_count > 0
                and max_count > 0
            ):
                if not np.isnan(ranges[cur]):
                    ranges[cur] = np.nan
                    cur += 1
                    blackout_count -= 1
                    max_count -= 1
                else:
                    break

    def timer_cb(self, event):

        now = rospy.Time.now()
        ls = LaserScan()
        ls.header.frame_id = self.TF_PREFIX + "laser_link"
        ls.header.stamp = now
        ls.angle_increment = self.ANGLE_STEP
        ls.angle_min = self.ANGLE_MIN
        ls.angle_max = self.ANGLE_MAX
        ls.range_min = self.MIN_RANGE_METERS
        ls.range_max = self.MAX_RANGE_METERS
        ls.intensities = []

        ranges = np.zeros(len(self.ANGLES) * 1, dtype=np.float32)

        laser_angle = utils.quaternion_to_angle(self.transform.rotation)
        laser_pose_x = self.transform.translation.x + self.x_offset * np.cos(laser_angle)
        laser_pose_y = self.transform.translation.y + self.x_offset * np.sin(laser_angle)

        range_pose = np.array(
            (laser_pose_x, laser_pose_y, laser_angle), dtype=np.float32
        ).reshape(1, 3)
        self.range_method.calc_range_repeat_angles(range_pose, self.ANGLES, ranges)
        self.noise_laser_scan(ranges)
        ls.ranges = ranges.tolist()
        self.laser_pub.publish(ls)

