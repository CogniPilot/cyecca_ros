#!/usr/bin/env python_cyecca
import sys
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
import rclpy.clock

from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, NavSatFix, MagneticField
from actuator_msgs.msg import Actuators
from nav_msgs.msg import Odometry

import cyecca
import casadi as ca


class QuadEstimator(Node):

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # parameters
        self.declare_parameter("dt", 0.005)
        self.set_parameters([
            Parameter("use_sim_time", Parameter.Type.BOOL, True),
        ])

        # actuators publication
        self.pub_odom = self.create_publisher(Odometry, "/odom", 10)
        self.dt = self.get_parameter("dt").value
        self.create_timer(
            timer_period_sec=self.dt,
            callback=self.update_odom,
            clock=self.get_clock())
        self.msg_odom = Odometry()
        self.msg_odom.header.frame_id = 'map'
        self.msg_odom.child_frame_id = 'base_link'

    def update_odom(self):
        t = self.get_clock().now().nanoseconds*1e-9
        self.msg_odom.header.stamp = self.get_clock().now().to_msg()
        self.msg_odom.pose.pose.position.x = np.sin(t)
        self.msg_odom.pose.pose.position.y = np.cos(t)
        self.msg_odom.pose.pose.position.z = 1.0
        self.msg_odom.pose.pose.orientation.x = 0.0
        self.msg_odom.pose.pose.orientation.y = 0.0
        self.msg_odom.pose.pose.orientation.z = 0.0
        self.msg_odom.pose.pose.orientation.w = 1.0
        self.pub_odom.publish(self.msg_odom)


def main(args):
    rclpy.init(args=args)
    estimator = QuadEstimator(node_name="quad_estimator")

    try:
        rclpy.spin(estimator)
    except KeyboardInterrupt as e:
        pass


if __name__ == "__main__":
    main(args=sys.argv[1:])
