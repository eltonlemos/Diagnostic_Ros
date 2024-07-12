#!/usr/bin/env python3
# -*- coding: utf-8 -*-


from random import random

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
import rclpy
from rclpy.clock import ROSClock
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

PKG = 'diagnostic_aggregator'

class DiagnosticTalker(Node):

    def __init__(self):
        super().__init__('diagnostic_talker')
        self.i = 0
        self.pub = self.create_publisher(DiagnosticArray,
                                         '/diagnostic_moog',
                                         qos_profile_system_default)
        timer_period = 1.0
        self.tmr = self.create_timer(timer_period, self.timer_callback)

        self.array = DiagnosticArray()
        self.array.status = [
            # lidar
            DiagnosticStatus(level=DiagnosticStatus.OK,
                             name='/ouster/points', message='OK'),
            DiagnosticStatus(level=DiagnosticStatus.OK,
                             name='/livox/lidar/pcd2', message='OK'),
            DiagnosticStatus(level=DiagnosticStatus.OK,
                             name='/livox/lidar', message='OK'),
]


    def timer_callback(self):
        self.array.header.stamp = ROSClock().now().to_msg()

        self.pub.publish(self.array)


def main(args=None):
    rclpy.init(args=args)

    node = DiagnosticTalker()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()