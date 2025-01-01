FROM ros:iron-perception

ARG ROS_VERSION=iron
ARG SWARM_WS=/root/swarm_ws

ENV FCU_URL=/dev/ttyTHS1:921600
ENV UWB_PORT=/dev/ttyTHS2
ENV VO_TOPIC=/d2vins/odometry
ENV VO_IMU_TOPIC=/d2vins/imu_propagation
ENV DRONE_ID=1

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y git ros-${ROS_VERSION}-mavros ros-${ROS_VERSION}-mavros-extras ros-${ROS_VERSION}-mavros-msgs \
      vim wget screen libglib2.0-dev python3-termcolor python3-matplotlib net-tools
RUN wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh
RUN chmod +x install_geographiclib_datasets.sh
RUN ./install_geographiclib_datasets.sh

RUN   mkdir -p ${SWARM_WS}/src/ && \
      cd ${SWARM_WS}/src/ && \
      git clone https://github.com/HKUST-Swarm/swarm_msgs.git -b ros2 && \
      git clone https://github.com/HKUST-Swarm/bspline.git -b ros2
      
COPY ./ ${SWARM_WS}/src/
COPY ./drone_commander/config/drone_commander_config.yaml /drone_commander_config.yaml
WORKDIR $SWARM_WS
SHELL ["/bin/bash", "-c"]
RUN   . "/opt/ros/${ROS_VERSION}/setup.sh" && \
      colcon build
COPY ./entrypoint.sh /

ENTRYPOINT ["/entrypoint.sh"]
