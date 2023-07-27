#!/usr/bin/env python_cyecca
import sys
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.lifecycle import LifecycleNode

from actuator_msgs.msg import Actuators
from nav_msgs.msg import Odometry

import casadi as ca
import cyecca


class QuadController(LifecycleNode):

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # parameters
        self.declare_parameter("dt", 0.01)
        self.set_parameters([
            Parameter("use_sim_time", Parameter.Type.BOOL, True),
        ])

        # actuators publication
        self.pub_actuators = self.create_publisher(Actuators, "/actuators", 10)
        self.dt = self.get_parameter("dt").value
        self.create_timer(
            timer_period_sec=self.dt,
            callback=self.update_actuators,
            clock=self.get_clock())
        self.msg_actuators = Actuators()
        self.msg_actuators.header.frame_id = 'base_link'
        self.msg_actuators.velocity.fromlist([0.0, 0.0, 0.0, 0.0])

    def update_actuators(self):
        self.msg_actuators.header.stamp = self.get_clock().now().to_msg()
        self.msg_actuators.velocity[0] = 1
        self.msg_actuators.velocity[1] = 2
        self.msg_actuators.velocity[2] = 3
        self.msg_actuators.velocity[3] = 4
        self.pub_actuators.publish(self.msg_actuators)


def main(args=None):
    rclpy.init(args=args)
    controller = QuadController(node_name="quad_controller")
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    controller.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
