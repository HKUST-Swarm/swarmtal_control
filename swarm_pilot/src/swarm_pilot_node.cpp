//
// Created by xuhao on 5/21/19.
//
#include <swarm_pilot.h>

int main(int argc, char** argv)
{

    ROS_INFO("swarm pilot\nIniting\n");
    ros::init(argc, argv, "swarm_pilot");
    ros::NodeHandle nh("swarm_pilot");
    SwarmPilot sp(nh);
    ros::spin();
}
