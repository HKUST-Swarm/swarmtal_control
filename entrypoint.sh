#!/bin/bash
# If argument1 is launch: to launch all nodes, else may in exec mode

source /root/swarm_ws/devel/setup.bash
echo "Launching with argument $1"
if [ "$1" == "launch" ]; then
    echo "Starting MAVROS on $FCU_URL, please set the FCU_URL in environment on docker, e.g. /dev/ttyUSB0:921600"
    nice --20 roslaunch mavros px4.launch fcu_url:=$FCU_URL &
    sleep 5
    echo "Launching UWB node"
    roslaunch inf_uwb_ros uwb_node_expo.launch self_id:=$DRONE_ID serial_name:=$UWB_PORT &
    echo "Starting swarmtal_control on $VO_TOPIC & $VO_IMU_TOPIC, please set the VO_TOPIC in environment, e.g. /d2vins/imu_propagation"
    nice --20 roslaunch drone_commander commander-px4.launch vo_topic:=$VO_TOPIC vo_imu_topic:=$VO_IMU_TOPIC \
        config_path:=/pos_control_param.yaml log_path:=/output/ &
    echo "Starting swarm_pilot with drone_id: $DRONE_ID"
    roslaunch swarm_pilot swarm_pilot.launch drone_id:=$DRONE_ID
    echo "PX4 and swarmtal_control started"
else if [ "$1" == "launch_sitl" ]; then
    echo "Checking if roscore is ready..."
    while ! rosnode list > /dev/null 2>&1; do
        echo "Waiting for roscore to be ready..."
        sleep 1
    done
    echo "Launching UWB node"
    roslaunch inf_uwb_ros uwb_node_expo.launch self_id:=$DRONE_ID serial_name:=$UWB_PORT &
    sleep 5
    echo "Starting swarmtal_control on $VO_TOPIC & $VO_IMU_TOPIC, please set the VO_TOPIC in environment, e.g. /d2vins/imu_propagation"
    roslaunch drone_commander commander-px4.launch vo_topic:=$VO_TOPIC vo_imu_topic:=$VO_IMU_TOPIC \
        config_path:=/pos_control_param.yaml log_path:=/output/ &
    echo "Starting swarm_pilot with drone_id: $DRONE_ID"
    roslaunch swarm_pilot swarm_pilot.launch drone_id:=$DRONE_ID
    echo "PX4 and swarmtal_control started"
else if [ "$1" == "cmd" ]; then
    # rosrun with all remaining arguments
    rosrun drone_commander drone_cmd.py ${@:2}
else if [ "$1" == "status" ]; then
    rosrun drone_commander drone_status.py $VO_TOPIC
else if [ "$1" == "plot" ]; then
    rosrun drone_commander swarmtal_csv_parser.py ${@:2}
else if [ "$1" == "test" ]; then
    echo "Input argument is ${@:2}"
else if [ "$1" == "bash" ]; then
    /bin/bash
else
    # exec with all arguments
    exec "$@"
fi
fi
fi
fi
fi
fi
fi