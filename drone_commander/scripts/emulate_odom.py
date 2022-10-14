#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry

if __name__ == "__main__":
    rospy.init_node('odom_gen', anonymous=True)
    odom_slow_pub = rospy.Publisher("/d2vins/odometry", Odometry, queue_size=1)
    odom_fast_pub = rospy.Publisher("/d2vins/imu_propagate", Odometry, queue_size=1)
    odom = Odometry()
    odom.header.frame_id = "world"
    odom.pose.pose.orientation.w = 1.0
    odom.pose.pose.orientation.x = 0.0
    odom.pose.pose.orientation.y = 0.0
    odom.pose.pose.orientation.z = 0.0
    odom.pose.pose.position.x = 0.1
    odom.pose.pose.position.y = 0.2
    odom.pose.pose.position.z = 0.3
    print("Start gen odom, publish to /d2vins/odometry and /d2vins/imu_propagate")

    rate = rospy.Rate(220)  # 20hz
    count = 0
    while not rospy.is_shutdown():
        try:
            odom.header.stamp = rospy.get_rostime()
            if count % 20 == 0:
                odom_slow_pub.publish(odom)
            odom_fast_pub.publish(odom)
            rate.sleep()
            count += 1
        except KeyboardInterrupt:
            exit(0)
