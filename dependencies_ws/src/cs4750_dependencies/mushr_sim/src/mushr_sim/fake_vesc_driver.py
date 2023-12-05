#!/usr/bin/env python

# Copyright (c) 2019, The Personal Robotics Lab, The MuSHR Team, The Contributors of MuSHR
# License: BSD 3-Clause. See LICENSE.md file in root directory.

import rospy
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped


class FakeVescDriver:
    def __init__(self):
        self.speed_sub = rospy.Subscriber(
            "commands/motor/speed", Float64, self.speed_cb
        )
        self.servo_position_sub = rospy.Subscriber(
            "commands/servo/position", Float64, self.servo_position_cb
        )
        self.state_pub = rospy.Publisher(
            "sensors/core", VescStateStamped, queue_size=10
        )
        self.servo_pub = rospy.Publisher(
            "sensors/servo_position_command", Float64, queue_size=10
        )

    def speed_cb(self, msg):
        vss = VescStateStamped()
        vss.header.stamp = rospy.Time.now()
        vss.state.speed = msg.data
        self.state_pub.publish(vss)

    def servo_position_cb(self, msg):
        out_msg = Float64()
        out_msg.data = msg.data
        self.servo_pub.publish(out_msg)

