# Introduction

This is a control package for DJI N3/PX4(in progress) high level position control using Visual Odometry or mocation capture.
It also provide some system iden function with combintation of my other great repo https://github.com/xuhao1/pyaircraftiden.

## Usage
### Build
Build for docker
```bash
make arm64
```

### Launch

Launch mavros with drone_commander, drone_position_control and swarm_pilot, change the FCU_URL, VO_TOPIC, DRONE_ID and $HOME/output to what you want.

```bash
docker run -it --rm --privileged \
    -v $HOME/output:/output/ -e FCU_URL="/dev/ttyTHS1:921600" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -e VO_TOPIC="/d2vins/imu_propagation" \
    -e DRONE_ID=1 \
    -e DISPLAY=$DISPLAY \
    --name swarmtal_control \
    buaaswarm/swarmtal_control launch
```

To verify: 

```bash
docker exec -it swarmtal_control /entrypoint.sh status 
```

Now you should see the battery voltage

```bash
[54.614s] VO False :[0.000, 0.000, 0.000] BAT: |----------|  0.0% :0.00V
```

To work with your custom control parameters (STONGLY RECOMMEND)
Move drone_positon_control/launch/pos_control_param.yaml to $HOME/SwarmConfig/pos_control_param.yaml and modify it for your own drone

```bash
docker run -it --rm --privileged --net=host \
    -v $HOME/output:/output/ -e FCU_URL="/dev/ttyTHS1:921600" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -v /home/nvidia/SwarmConfig/pos_control_param.yaml:/pos_control_param.yaml \
    -e VO_TOPIC="/Odometry" \
    -e VO_IMU_TOPIC="/d2vins/imu_propagation" \
    -e DRONE_ID=1 \
    -e DISPLAY=$DISPLAY \
    --name swarmtal_control \
    buaaswarm/swarmtal_control launch
```

### Testing
First thing you may want to do is visualize its status

```bash
docker exec -it swarmtal_control /entrypoint.sh status 
```

Of-course, try to arm and control it, simply by sending command to drone_commander by command:

```bash
docker exec -it swarmtal_control /entrypoint.sh cmd [arm,takeoff,landing,....]
```

And to visualize log file

```bash
docker exec -it swarmtal_control /entrypoint.sh plot /output/name-of-log-file.csv
```

## Interface

The only interface is sending message to topic "/drone_commander/onboard_command".

drone_cmd.py provide a great example for usage including takeoff, flyto somewhere landing and also do a sweep frequency experiment.

```
uint32 CTRL_POS_COMMAND=0
uint32 CTRL_VEL_COMMAND=1
uint32 CTRL_ATT_COMMAND=2
uint32 CTRL_MISSION_LOAD_COMMAND=3
uint32 CTRL_MISSION_END_COMMAND=4
uint32 CTRL_TAKEOF_COMMAND=5
uint32 CTRL_LANDING_COMMAND=6
uint32 CTRL_HOVER_COMMAND=7
uint32 CTRL_ARM_COMMAND=8

uint32 command_type

# For POS param1 2 3 is x y z * 10000 
# 4 is yaw*10000 if yaw =666666, then use last yawsp
# 567 is vel feedforward
# 8,9,10 is acc feeforward

# For VEL param1 2 3 is x y z * 10000 
# 4 is yaw*10000 if yaw =666666, then use last yawsp

# For ATT 
# param1 2 3 4 is roll pitch yaw vz * 10000 
# 5 (>0 use thrust else z is velz)
# 6 (>0 use yaw else yawrate) 
# For TAKEoff param 1 is takeoff height * 10000
# For arm param = 0 is disarm param>0 is arm
# For mission param1-7 is  mission id

int32 param1
int32 param2
int32 param3
int32 param4
int32 param5
int32 param6
int32 param7
int32 param8
int32 param9
int32 param10

```

## Debugging
If you want to change some scripts or entrypoints

```bash
docker run -it --rm --privileged \
    -v $HOME/output:/output/ -e FCU_URL="/dev/ttyTHS1:921600" \
    -v ./:/root/swarm_ws/src/ \
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -e VO_TOPIC="/d2vins/imu_propagation" \
    -e DRONE_ID=1 \
    -e DISPLAY=$DISPLAY \
    --name swarmtal_control \
    buaaswarm/swarmtal_control launch
```

## Save
```bash
docker save buaaswarm/swarmtal_control | gzip > swarmtal_control.tgz
```

## Load

```bash
gunzip -c swarmtal_control.tgz | docker load
```

## License
You can use this software freely. But if this crash your drone, no body will help you.
