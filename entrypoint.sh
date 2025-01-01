#!/bin/bash
# If argument1 is launch: to launch all nodes, else may in exec mode

source /root/swarm_ws/install/setup.bash
echo "Launching with argument $1"
if [ "$1" == "launch" ]; then
    sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev eth0
    echo "Starting MAVROS on $FCU_URL, please set the FCU_URL in environment on docker, e.g. /dev/ttyUSB0:921600"
    nice --20 ros2 launch mavros px4.launch fcu_url:=$FCU_URL namespace:=/uav$DRONE_ID &
    echo "Starting swarmtal_control on $VO_TOPIC & $VO_IMU_TOPIC, please set the VO_TOPIC in environment, e.g. /d2vins/imu_propagation"
    nice --20 ros2 launch drone_commander px4.launch.py vo_topic:=$VO_TOPIC vo_imu_topic:=$VO_IMU_TOPIC  drone_id:=$DRONE_ID \
        config_path:=/drone_commander_config.yaml enable_odom_transformer:=false
else if [ "$1" == "launch_sitl" ]; then
    sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev eth0
    echo "Starting swarmtal_control on $VO_TOPIC & $VO_IMU_TOPIC, please set the VO_TOPIC in environment, e.g. /d2vins/imu_propagation"
    nice --20 ros2 launch drone_commander px4.launch.py vo_topic:=$VO_TOPIC vo_imu_topic:=$VO_IMU_TOPIC drone_id:=$DRONE_ID \
        config_path:=/drone_commander_config.yaml enable_odom_transformer:=true
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