#!/usr/bin/env python_cyecca
import sys
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from rclpy.lifecycle import LifecycleNode
import rclpy.clock

from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, NavSatFix, MagneticField
from actuator_msgs.msg import Actuators

import cyecca
import casadi as ca


class QuadSimulator(LifecycleNode):

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # parameters
        self.declare_parameter("dt", 0.005)
        self.declare_parameter("speed", 10)
        self.declare_parameter("monitor_steps", 1000)
        self.declare_parameter("dt_imu", 0.005)
        self.declare_parameter("dt_gnss", 0.1)
        self.declare_parameter("dt_mag", 0.02)
        self.set_parameters([
            Parameter("use_sim_time", Parameter.Type.BOOL, True),
        ])

        # clock
        self.pub_clock = self.create_publisher(Clock, "/clock", 10)
        self.dt = self.get_parameter("dt").value
        self.speed = self.get_parameter("speed").value
        self.monitor_steps = self.get_parameter("monitor_steps").value
        self.system_clock = rclpy.clock.Clock()
        self.create_timer(
            timer_period_sec=self.dt/self.speed,
            callback=self.update_clock,
            clock=self.system_clock)
        self.i = 0
        self.system_time_prev = self.system_clock.now()
        self.msg_clock = Clock()

        # imu
        self.pub_imu = self.create_publisher(Imu, "/imu", 10)
        self.dt_imu = self.get_parameter("dt_imu").value
        self.create_timer(
            timer_period_sec=self.dt_imu,
            callback=self.update_imu,
            clock=self.get_clock())
        self.msg_imu = Imu()
        self.msg_imu.header.frame_id = 'base_link'

        # gnss
        self.pub_gnss = self.create_publisher(NavSatFix, "/gnss", 10)
        self.dt_gnss = self.get_parameter("dt_gnss").value
        self.create_timer(
            timer_period_sec=self.dt_gnss,
            callback=self.update_gnss,
            clock=self.get_clock())
        self.msg_gnss = NavSatFix()
        self.msg_gnss.header.frame_id = 'base_link'
        self.msg_gnss.position_covariance = np.diag([1.0, 1.0, 1.0]).reshape(-1)
        self.msg_gnss.position_covariance_type = 1 # diagonal known

        # mag
        self.pub_mag = self.create_publisher(MagneticField, "/mag", 10)
        self.dt_mag = self.get_parameter("dt_mag").value
        self.create_timer(
            timer_period_sec=self.dt_mag,
            callback=self.update_mag,
            clock=self.get_clock())
        self.msg_mag = MagneticField()
        self.msg_mag.header.frame_id = 'base_link'
        self.msg_mag.magnetic_field_covariance = np.diag([1.0, 1.0, 1.0]).reshape(-1)

        # control
        self.motors = [0, 0, 0, 0]
        self.sub_actuators = self.create_subscription(
                Actuators, "/actuators", self.callback_actuators, 10)


    def update_clock(self):
        t = self.i * self.dt
        seconds = int(t)
        nanoseconds = int(1e9*(t - seconds))
        self.msg_clock.clock = Time(
            seconds=seconds,
            nanoseconds=nanoseconds,
            clock_type=rclpy.clock.ClockType.ROS_TIME).to_msg()
        self.pub_clock.publish(self.msg_clock)

        if self.i > 0 and self.i % self.monitor_steps == 0: 
            now = self.system_clock.now()
            sim_rate = self.monitor_steps*1e9*self.dt/(now - self.system_time_prev).nanoseconds
            self.system_time_prev = now
            self.get_logger().info(f"sim rate: {sim_rate:f}", throttle_duration_sec=1.0)

        # increment time for next loop
        self.i += 1

        # update dynamics
        self.update_dynamcis()

    def update_dynamcis(self):
        pass

    def update_imu(self):
        self.msg_imu.header.stamp = self.get_clock().now().to_msg()
        self.msg_imu.angular_velocity.x = 1.0
        self.msg_imu.angular_velocity.y = 2.0
        self.msg_imu.angular_velocity.z = 3.0
        self.pub_imu.publish(self.msg_imu)

    def update_gnss(self):
        self.msg_gnss.header.stamp = self.get_clock().now().to_msg()
        self.msg_gnss.altitude = 0.0
        self.msg_gnss.latitude = 1.0
        self.msg_gnss.longitude = 2.0
        self.pub_gnss.publish(self.msg_gnss)

    def update_mag(self):
        self.msg_mag.header.stamp = self.get_clock().now().to_msg()
        self.msg_mag.magnetic_field.x = 1.0
        self.msg_mag.magnetic_field.y = 2.0
        self.msg_mag.magnetic_field.z = 2.0
        self.pub_mag.publish(self.msg_mag)

    def callback_actuators(self, msg: Actuators):
        for i in range(4):
            self.motors[i] = msg.velocity[i]


def main(args=None):
    try:
        rclpy.init(args=args)
        simulator = QuadSimulator(node_name="quad_simulator")
        rclpy.spin(simulator)
    except KeyboardInterrupt:
        pass
    simulator.destroy_node()
    rclpy.try_shutdown()

if __name__ == "__main__":
    main()
