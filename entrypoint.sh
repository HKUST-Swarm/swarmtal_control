#!/bin/bash
# Arguments: $1: fcu_url, $2: vo_topic
source /opt/ros/noetic/setup.bash
source /root/swarm_ws/devel/setup.bash
echo "Starting MAVROS on $FCU_URL, please set the FCU_URL in environment on docker, e.g. /dev/ttyUSB0:921600"
nice --20 roslaunch mavros px4.launch fcu_url:=$FCU_URL &
sleep 5
echo "Starting swarmtal_control on $VO_TOPIC, please set the VO_TOPIC in environment, e.g. /d2vins/imu_propagation"
nice --20 roslaunch drone_commander commander-px4.launch vo_topic:=$VO_TOPIC \
    config_path:=/pos_control_param.yaml log_path:=/output/ &
echo "Starting swarm_pilot with drone_id: $DRONE_ID, please set the DRONE_ID in environment"
roslaunch swarm_pilot swarm_pilot.launch drone_id:=$DRONE_ID
echo "PX4 and swarmtal_control started"