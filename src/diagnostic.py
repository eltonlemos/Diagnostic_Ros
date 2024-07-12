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
                                         '/diagnostics_moog',
                                         qos_profile_system_default)
        timer_period = 1.0
        self.tmr = self.create_timer(timer_period, self.timer_callback)

        self.array = DiagnosticArray()
        self.array.status = [
            # lidar
            # DiagnosticStatus(level=DiagnosticStatus.OK,
            #                  name='/ouster/points', message='OK'),
            # DiagnosticStatus(level=DiagnosticStatus.OK,
            #                  name='/livox/lidar/pcd2', message='OK'),
            # DiagnosticStatus(level=DiagnosticStatus.OK,
            #                  name='/livox/lidar', message='OK'),
            DiagnosticStatus(level=DiagnosticStatus.OK,
                             name='/arms/left/motor', message='OK'),
            DiagnosticStatus(level=DiagnosticStatus.OK,
                             name='/arms/right/motor', message='OK'),
            DiagnosticStatus(level=DiagnosticStatus.OK,
                             name='/legs/left/motor', message='OK'),
            DiagnosticStatus(level=DiagnosticStatus.OK,
                             name='/legs/right/motor', message='OK'),

            # Sensors
            DiagnosticStatus(level=DiagnosticStatus.OK,
                             name='/sensors/left/cam', message='OK'),
            DiagnosticStatus(level=DiagnosticStatus.OK,
                             name='/sensors/right/cam', message='OK'),
            DiagnosticStatus(level=DiagnosticStatus.OK,
                             name='/sensors/front/cam', message='OK'),
            DiagnosticStatus(level=DiagnosticStatus.OK,
                             name='/sensors/rear/cam', message='OK'),

            # Optional
            DiagnosticStatus(level=DiagnosticStatus.OK,
                             name='/optional/runtime/analyzer', message='OK'),
        ]


    def timer_callback(self):
        self.array.header.stamp = ROSClock().now().to_msg()

        # Random diagnostics status
        level = random()
        self.array.status[1].level = DiagnosticStatus.OK
        self.array.status[1].message = 'OK'
        self.array.status[3].level = DiagnosticStatus.OK
        self.array.status[3].message = 'OK'
        self.array.status[4].level = DiagnosticStatus.OK
        self.array.status[4].message = 'OK'
        if level > .5:
            self.array.status[1].level = DiagnosticStatus.WARN
            self.array.status[1].message = 'Warning'
        if level > .7:
            self.array.status[3].level = DiagnosticStatus.WARN
            self.array.status[3].message = 'Warning'
        if level > .95:
            self.array.status[4].level = DiagnosticStatus.ERROR
            self.array.status[4].message = 'Error'

        self.pub.publish(self.array)


def main(args=None):
    rclpy.init(args=args)

    node = DiagnosticTalker()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()