#!/usr/bin/env python
import rospy
from swarmtal_msgs.msg import *
from nav_msgs.msg import Odometry

odom = None

def on_odometry_recv(_odom):
    global odom
    odom = _odom

def onboard_state_gen():
    print("Will generate simulation data for swarm_commander_state")
    pub = rospy.Publisher('/drone_commander/swarm_commander_state', drone_commander_state, queue_size=10)
    sub = rospy.Subscriber("/vins_estimator/odometry", Odometry, on_odometry_recv, queue_size=1)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        state = drone_commander_state()
        if odom is not None:
            state.vo_valid = True
            state.pos = odom.pose.pose.position
        else:
            state.vo_valid = False
        state.bat_vol = 17.0
        state.bat_remain = 1000
        pub.publish(state)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node("pilot_sim_gen", anonymous=True)
    onboard_state_gen()
