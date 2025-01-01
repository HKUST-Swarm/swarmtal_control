#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import argparse
import sys
import math
import time
from termcolor import colored
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry

from swarmtal_msgs.msg import DroneOnboardCommand

class DroneStatus(Node):
    def __init__(self, vo_topic):
        super().__init__('drone_status')
        
        # 初始化变量
        self.battery_voltage = 0.0
        self.vo_position = Vector3()
        self.vo_avail = False
        self.last_vo_time = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec / 1e9

        # 创建订阅者
        self.battery_sub = self.create_subscription(
            BatteryState,
            '/mavros/battery',
            self.on_battery_status,
            10
        )
        self.vo_sub = self.create_subscription(
            Odometry,
            vo_topic,
            self.on_vo_msg,
            10
        )

        self.get_logger().info("Subscribed to /mavros/battery and {}".format(vo_topic))

        # 创建定时器，频率为10Hz（0.1秒）
        timer_period = 0.1  # 秒
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def on_battery_status(self, msg):
        self.battery_voltage = msg.voltage

    def on_vo_msg(self, msg):
        self.vo_avail = True
        self.vo_position = msg.pose.pose.position
        self.last_vo_time = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec / 1e9

    def battery_to_percent(self, bat):
        if bat > 14.8:
            return (bat - 14.8)/(16.8-14.8)*0.5 + 0.5
        if 0 < bat < 14.8:
            return (bat - 14.4)/(14.8 - 14.4)*0.5
        return 0.0

    def print_battery_level(self, battery_level, total=100, prefix='', suffix='', length=100, fill='█'):
        percent = ("{0:4.1f}%").format(100 * (battery_level / float(total)))
        filled_length = int(length * battery_level // total)
        bar = "|" + fill * filled_length + '-' * (length - filled_length) + "|"
        bar_color = "blue"
        if battery_level < 30:
            bar_color = "red"
        elif battery_level < 70:
            bar_color = "yellow"
        elif battery_level < 95:
            bar_color = "green"

        print('\r%s %s %s %s' % (prefix, colored(bar, bar_color), colored(percent, bar_color), suffix), end='\r')
        sys.stdout.flush()

    def timer_callback(self):
        current_time = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec / 1e9
        prefix = "[{:7.3f}s]".format(current_time % 1000)

        if self.vo_avail and (current_time - self.last_vo_time) <= 0.2:
            vo_color = "green"
            vo_str = " VO {} :[{:5.3f}, {:5.3f}, {:5.3f}]".format(self.vo_avail, 
                                                                   self.vo_position.x, 
                                                                   self.vo_position.y, 
                                                                   self.vo_position.z)
        else:
            vo_color = "red"
            vo_str = " VO {} :[{:5.3f}, {:5.3f}, {:5.3f}]".format(False, 0.0, 0.0, 0.0)
            self.vo_avail = False  # 如果超过时间阈值，认为VO不可用

        prefix = prefix + colored(vo_str, vo_color) + " BAT:"
        bat_percent = self.battery_to_percent(self.battery_voltage) * 100
        suffix = ":{:4.2f}V".format(self.battery_voltage)
        self.print_battery_level(bat_percent, 100, prefix, suffix, length=10)

def main(args=None):
    parser = argparse.ArgumentParser(description='A simple command tool for monitoring drone status.')
    parser.add_argument('--vo_topic', type=str, default='/d2vins/odometry', 
                        help='Visual Odometry topic to subscribe to.')
    args = parser.parse_args()

    rclpy.init()

    drone_status_node = DroneStatus(vo_topic=args.vo_topic)

    try:
        rclpy.spin(drone_status_node)
    except KeyboardInterrupt:
        pass
    finally:
        drone_status_node.destroy_node()
        rclpy.shutdown()
        print()

if __name__ == "__main__":
    main()
