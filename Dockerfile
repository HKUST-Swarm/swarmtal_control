# FROM --platform=linux/arm64 ros:noetic-perception-focal
FROM ros:noetic-perception-focal

ARG ROS_VERSION=noetic
ENV DEBIAN_FRONTEND=noninteractive
ENV SWARM_WS=/root/swarm_ws

RUN apt-get update && apt-get install -y git ros-${ROS_VERSION}-mavros ros-${ROS_VERSION}-mavros-extras ros-${ROS_VERSION}-mavros-msgs vim wget screen
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
RUN chmod +x install_geographiclib_datasets.sh
RUN ./install_geographiclib_datasets.sh

#Build D2SLAM
RUN   mkdir -p ${SWARM_WS}/src/ && \
      cd ${SWARM_WS}/src/ && \
      git clone https://github.com/HKUST-Swarm/swarm_msgs.git -b D2SLAM
COPY ./ ${SWARM_WS}/src/
WORKDIR $SWARM_WS
SHELL ["/bin/bash", "-c"]
RUN   . "/opt/ros/${ROS_VERSION}/setup.sh" && \
      catkin_make -DCMAKE_BUILD_TYPE=Release
COPY ./entrypoint.sh /
COPY ./drone_position_control/launch/pos_control_param.yaml /

ENTRYPOINT ["/entrypoint.sh"]
