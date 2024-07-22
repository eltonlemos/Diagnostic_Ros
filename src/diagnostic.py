#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2008, Willow Garage, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# This file is originally from:
# https://github.com/ros/ros_comm/blob/6e5016f4b2266d8a60c9a1e163c4928b8fc7115e/tools/rostopic/src/rostopic/__init__.py

from collections import defaultdict

import functools
import math
import threading
import time

import rclpy

from rclpy.clock import Clock
from rclpy.clock import ROSClock
from rclpy.clock import ClockType
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from ros2cli.node.direct import add_arguments as add_direct_node_arguments
from ros2cli.node.direct import DirectNode
from ros2topic.api import get_msg_class
from diagnostic_msgs.msg import KeyValue
# from ros2topic.api import positive_int
from ros2topic.api import TopicNameCompleter
from ros2topic.verb import VerbExtension
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
import yaml
from rclpy.qos import qos_profile_system_default
from argparse import ArgumentTypeError
# from rclpy.qos import qos_profile_default
DEFAULT_WINDOW_SIZE = 10000
PKG = 'diagnostic_aggregator'

def positive_int(string):
    try:
        value = int(string)
    except ValueError:
        value = -1
    if value <= 0:
        raise ArgumentTypeError('value must be a positive integer')
    return value

class HzVerb(VerbExtension):
    """Print the average publishing rate to screen."""

    # def add_arguments(self, parser, cli_name):
    #     arg = parser.add_argument(
    #         '--config',"-c",
    #         dest='path', type = str, default = "config/topics.yaml"
    #         help="Name of the ROS topic to listen to (e.g. '/chatter')")
    #     arg.completer = TopicNameCompleter(
    #         include_hidden_topics_key='include_hidden_topics')
    #     parser.add_argument(
    #         '--window', '-w',
    #         dest='window_size', type=positive_int, default=DEFAULT_WINDOW_SIZE,
    #         help='window size, in # of messages, for calculating rate '
    #              '(default: %d)' % DEFAULT_WINDOW_SIZE, metavar='WINDOW')
    #     parser.add_argument(
    #         '--filter',
    #         dest='filter_expr', default=None,
    #         help='only measure messages matching the specified Python expression', metavar='EXPR')
    #     parser.add_argument(
    #         '--wall-time',
    #         dest='use_wtime', default=False, action='store_true',
    #         help='calculates rate using wall time which can be helpful'
    #              ' when clock is not published during simulation')
    #     add_direct_node_arguments(parser)

    def main(self, *, args):
        return main(args)


def main(args):
    # topics = args.topic_name
    # topics = ["/navsatfix", "/livox/lidar", "/diagnostics" ]
    # error_rates = [100, 10, 1000]
    # warning_rates = [150, 15, 1500]
    import pathlib
    print(pathlib.Path().resolve())
    path = args.path
    with open(path, 'r') as f:
        data = yaml.load(f, Loader=yaml.SafeLoader)
    
    topics = data["topics"]
    error_rates = data["error_rates"]
    warning_rates = data["warning_rates"]

    if args.filter_expr:
        def expr_eval(expr):
            def eval_fn(m):
                return eval(expr)
            return eval_fn
        filter_expr = expr_eval(args.filter_expr)
    else:
        filter_expr = None

    with DirectNode(args) as node:
        _rostopic_hz(node.node, topics, error_rates, warning_rates, window_size=args.window_size, filter_expr=filter_expr,
                     use_wtime=args.use_wtime)


class ROSTopicHz(Node):
    """ROSTopicHz receives messages for a topic and computes frequency."""

    def __init__(self, node, error_rates, warning_rates, window_size, filter_expr=None, use_wtime=False):
        super().__init__('diagnostic_talker')
        self.lock = threading.Lock()
        self.last_printed_tn = 0
        self.msg_t0 = -1
        self.msg_tn = 0
        self.times = []
        self._last_printed_tn = defaultdict(int)
        self._msg_t0 = defaultdict(lambda: -1)
        self._msg_tn = defaultdict(int)
        self._times = defaultdict(list)
        self.filter_expr = filter_expr
        self.use_wtime = use_wtime

        self.window_size = window_size
        self.error_rates = error_rates
        self.warning_rates = warning_rates
        self.pub = self.create_publisher(DiagnosticArray,
                                    '/diagnostics',
                                    qos_profile_system_default)

        # Clock that has support for ROS time.
        self._clock = node.get_clock()
        self.diagnostic_array = None


    def get_last_printed_tn(self, topic=None):
        if topic is None:
            return self.last_printed_tn
        return self._last_printed_tn[topic]

    def set_last_printed_tn(self, value, topic=None):
        if topic is None:
            self.last_printed_tn = value
        self._last_printed_tn[topic] = value

    def get_msg_t0(self, topic=None):
        if topic is None:
            return self.msg_t0
        return self._msg_t0[topic]

    def set_msg_t0(self, value, topic=None):
        if topic is None:
            self.msg_t0 = value
        self._msg_t0[topic] = value

    def get_msg_tn(self, topic=None):
        if topic is None:
            return self.msg_tn
        return self._msg_tn[topic]

    def set_msg_tn(self, value, topic=None):
        if topic is None:
            self.msg_tn = value
        self._msg_tn[topic] = value

    def get_times(self, topic=None):
        if topic is None:
            return self.times
        return self._times[topic]

    def set_times(self, value, topic=None):
        if topic is None:
            self.times = value
        self._times[topic] = value

    def callback_hz(self, m, topic=None):
        """
        Calculate interval time.

        :param m: Message instance
        :param topic: Topic name
        """
        # ignore messages that don't match filter
        if self.filter_expr is not None and not self.filter_expr(m):
            return
        with self.lock:
            # Uses ROS time as the default time source and Walltime only if requested
            curr_rostime = self._clock.now() if not self.use_wtime else \
                Clock(clock_type=ClockType.SYSTEM_TIME).now()

            # time reset
            if curr_rostime.nanoseconds == 0:
                if len(self.get_times(topic=topic)) > 0:
                    print('time has reset, resetting counters')
                    self.set_times([], topic=topic)
                return

            curr = curr_rostime.nanoseconds
            msg_t0 = self.get_msg_t0(topic=topic)
            if msg_t0 < 0 or msg_t0 > curr:
                self.set_msg_t0(curr, topic=topic)
                self.set_msg_tn(curr, topic=topic)
                self.set_times([], topic=topic)
            else:
                self.get_times(topic=topic).append(curr - self.get_msg_tn(topic=topic))
                self.set_msg_tn(curr, topic=topic)

            if len(self.get_times(topic=topic)) > self.window_size:
                self.get_times(topic=topic).pop(0)

    def get_hz(self, topic=None):
        """
        Calculate the average publishing rate.

        :param topic: topic name, ``list`` of ``str``
        :returns: tuple of stat results
            (rate, min_delta, max_delta, standard deviation, window number)
            None when waiting for the first message or there is no new one
        """
        if not self.get_times(topic=topic):
            return
        elif self.get_msg_tn(topic=topic) == self.get_last_printed_tn(topic=topic):
            return
        with self.lock:
            # Get frequency every one minute
            times = self.get_times(topic=topic)
            n = len(times)
            mean = sum(times) / n
            rate = 1. / mean if mean > 0. else 0

            # std dev
            std_dev = math.sqrt(sum((x - mean)**2 for x in times) / n)

            # min and max
            max_delta = max(times)
            min_delta = min(times)

            self.set_last_printed_tn(self.get_msg_tn(topic=topic), topic=topic)

        return rate, min_delta, max_delta, std_dev, n

    def print_hz(self, topics=None):
        """Print the average publishing rate to screen."""

        def get_format_hz(stat):
            return stat[0] * 1e9, stat[1] * 1e-9, stat[2] * 1e-9, stat[3] * 1e-9, stat[4]

        # if len(topics) == 1:
        #     ret = self.get_hz(topics[0])
        #     if ret is None:
        #         return
        #     rate, min_delta, max_delta, std_dev, window = get_format_hz(ret)
        #     print('average rate: %.3f\n\tmin: %.3fs max: %.3fs std dev: %.5fs window: %s'
        #           % (rate, min_delta, max_delta, std_dev, window))
        #     return

        # monitoring multiple topics' hz
        header = ['topic', 'rate', 'min_delta', 'max_delta', 'std_dev', 'window']
        for topic in range(len(topics)):
            hz_stat = self.get_hz(topics[topic])
            if hz_stat is None:
                self.diagnostic_array.status[topic].level = DiagnosticStatus.ERROR
                self.diagnostic_array.status[topic].message = str(topics[topic])+ " is not being published"
                continue
            rate, min_delta, max_delta, std_dev, window = get_format_hz(hz_stat)
            if rate <= self.error_rates[topic]:
                level = DiagnosticStatus.ERROR
                message = str(topics[topic])+ " frequency is critically low"
            elif rate <= self.warning_rates[topic]:
                level = DiagnosticStatus.WARN
                message = str(topics[topic])+ " frequency is low"
            else:
                level = DiagnosticStatus.OK
                message = str(topics[topic])+ " is OK"
            self.diagnostic_array.status[topic].level = level
            self.diagnostic_array.status[topic].message = message
            self.diagnostic_array.status[topic].values[0].value = str(rate)
            self.diagnostic_array.status[topic].values[1].value = str(min_delta)
            self.diagnostic_array.status[topic].values[2].value = str(max_delta)
            self.diagnostic_array.status[topic].values[3].value = str(std_dev)
            self.diagnostic_array.status[topic].values[4].value = str(window)
        self.pub.publish(self.diagnostic_array)


def _rostopic_hz(node, topics, error_rates, warning_rates, window_size=DEFAULT_WINDOW_SIZE, filter_expr=None, use_wtime=False):
    """
    Periodically print the publishing rate of a topic to console until shutdown.

    :param topics: list of topic names, ``list`` of ``str``
    :param window_size: number of messages to average over, -1 for infinite, ``int``
    :param filter_expr: Python filter expression that is called with m, the message instance
    """
    # pause hz until topic is published
    rt = ROSTopicHz(node, error_rates, warning_rates, window_size, filter_expr=filter_expr, use_wtime=use_wtime)
    topics_len = len(topics)
    topic_dict = {}
    blank_list = []
    rt.diagnostic_array = DiagnosticArray()
    rt.diagnostic_array.status = []
    for i,topic in enumerate(topics):
        msg_class = get_msg_class(
            node, topic, blocking=False, include_hidden_topics=True)
        rt.diagnostic_array.status.append(DiagnosticStatus(level=DiagnosticStatus.WARN,
            name=topic, message= str(topic)+" yet to be published", values=[KeyValue(),KeyValue(),KeyValue(),KeyValue(),KeyValue(),KeyValue(),KeyValue()]))
        rt.diagnostic_array.status[-1].values[0].key = "Rate"
        rt.diagnostic_array.status[-1].values[1].key = "Min_Delta"
        rt.diagnostic_array.status[-1].values[2].key = "Max_Delta"
        rt.diagnostic_array.status[-1].values[3].key = "Std_Dev"
        rt.diagnostic_array.status[-1].values[4].key = "Window"
        rt.diagnostic_array.status[-1].values[5].key = "Err_Rate"
        rt.diagnostic_array.status[-1].values[6].key = "Warn_Rate"
        rt.diagnostic_array.status[-1].values[5].value = str(error_rates[i])
        rt.diagnostic_array.status[-1].values[6].value = str(warning_rates[i])

        # rt.diagnostic_array.status.append(DiagnosticStatus(level=DiagnosticStatus.ERROR,
        #                     name=topic, message='NULL'))
        # rt.diagnostic_array.status.append(DiagnosticStatus(level=DiagnosticStatus.ERROR,
        #                     name=topic, message='NULL'))
        # if msg_class is None:
        #     node.destroy_node()
        #     return
        topic_dict[topic] = msg_class
        if msg_class is None:
            blank_list.append(topic)
            continue
        node.create_subscription(
            msg_class,
            topic,
            functools.partial(rt.callback_hz, topic=topic),
            qos_profile_sensor_data)
        if topics_len > 1:
            print('Subscribed to [%s]' % topic)

    try:
        def thread_func():
            while rclpy.ok():
                rt.print_hz(topics)
                time.sleep(1.0)
        
        def thread_sub(topic):
        
            msg_class = get_msg_class(
                node, topic, blocking=True, include_hidden_topics=True)

            if msg_class is None:
                node.destroy_node()
                return
            topic_dict[topic] = msg_class
            node.create_subscription(
                msg_class,
                topic,
                functools.partial(rt.callback_hz, topic=topic),
                qos_profile_sensor_data)
            
            if topics_len > 1:
                print('Subscribed to [%s]' % topic)

        print_thread = threading.Thread(target=thread_func)
        print_thread.start()
        sub_thread_arr = []
        for topic in blank_list:
            sub_thread_arr.append(threading.Thread(target=thread_sub, args=(topic,)))
            sub_thread_arr[-1].start()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        print_thread.join()
        for thread in sub_thread_arr:
            thread.join()

if __name__ == '__main__':
    import sys
    from argparse import ArgumentParser

    parser = ArgumentParser(description='Print the average publishing rate to screen.')
    parser.add_argument(
    '--config',"-c",
    dest='path', type = str, default = "src/diagnostic_moog/config/topics.yaml",
    help="Name of the ROS topic to listen to (e.g. '/chatter')")
    parser.add_argument(
    '--window', '-w',
    dest='window_size', type=int, default=DEFAULT_WINDOW_SIZE,
    help='window size, in # of messages, for calculating rate (default: %d)' % DEFAULT_WINDOW_SIZE, metavar='WINDOW')
    parser.add_argument(
        '--filter',
        dest='filter_expr', default=None,
        help='only measure messages matching the specified Python expression', metavar='EXPR')
    parser.add_argument(
        '--wall-time',
        dest='use_wtime', default=False, action='store_true',
        help='calculates rate using wall time which can be helpful when clock is not published during simulation')
    parser.add_argument(
        '--ros-args',
        dest='not_used', default=False, action='store_true',
        help='not used')

    args = parser.parse_args()

    # rclpy.init(args=None)
    main(args)