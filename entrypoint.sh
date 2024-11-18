#!/bin/bash
# Arguments: $1: fcu_url, $2: vo_topic
source /opt/ros/noetic/setup.bash
source /root/swarm_ws/devel/setup.bash
echo "Starting MAVROS on $1"
nice --20 roslaunch mavros px4.launch fcu_url:=$1 &
sleep 3
echo "Starting swarmtal_control on $2"
nice --20 roslaunch drone_commander commander-px4.launch vo_topic:=$2 \
    config_path:=/pos_control_param.yaml log_path:=/output/
echo "PX4 and swarmtal_control started"