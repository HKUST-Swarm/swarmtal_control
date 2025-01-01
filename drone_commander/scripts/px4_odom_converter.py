#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomTransformer(Node):
    def __init__(self):
        super().__init__('odom_transformer')
        self.get_logger().info("Initializing odom_transformer node...")

        # 订阅 ~odom_in 话题
        self.odom_subscriber = self.create_subscription(
            Odometry,
            'odom_in',
            self.odometry_callback,
            10
        )

        # 发布 ~odom_out 话题
        self.odom_publisher = self.create_publisher(
            Odometry,
            'odom_out',
            10
        )

        self.get_logger().info("Listening to 'odom_in' and will publish transformed odometry to 'odom_out'")

    def odometry_callback(self, msg: Odometry):
        """
        Callback: transform incoming odometry data by rotating velocity axes 
        (for example, [-vel_y, vel_x, vel_z]) and re-publish with some modifications.
        """
        pos_x = msg.pose.pose.position.x
        pos_y = msg.pose.pose.position.y
        pos_z = msg.pose.pose.position.z

        # 这里原先只是给 position 做了简单的数组赋值，不做旋转，保持不变
        # (你可以在此进行更多的变换)
        modified_position = [pos_x, pos_y, pos_z]

        vel_x = msg.twist.twist.linear.x
        vel_y = msg.twist.twist.linear.y
        vel_z = msg.twist.twist.linear.z

        # 旋转 vel_y -> -vel_y, vel_x -> vel_x
        modified_velocity = [-vel_y, vel_x, vel_z]

        # 构造新的里程计消息
        modified_odom = Odometry()
        modified_odom.header = msg.header
        # 修改 frame_id (可根据需求)
        modified_odom.header.frame_id = "world"

        # 位置保持不变
        modified_odom.pose.pose.position.x = modified_position[0]
        modified_odom.pose.pose.position.y = modified_position[1]
        modified_odom.pose.pose.position.z = modified_position[2]
        modified_odom.pose.pose.orientation = msg.pose.pose.orientation  # 姿态不变

        # 速度使用我们的变换后的值
        modified_odom.twist.twist.linear.x = modified_velocity[0]
        modified_odom.twist.twist.linear.y = modified_velocity[1]
        modified_odom.twist.twist.linear.z = modified_velocity[2]

        # 角速度保持不变
        modified_odom.twist.twist.angular = msg.twist.twist.angular

        # 发布变换后的里程计
        self.odom_publisher.publish(modified_odom)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
