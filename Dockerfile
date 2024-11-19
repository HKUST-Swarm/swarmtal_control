FROM ros:noetic-perception-focal

ARG ROS_VERSION=noetic
ARG SWARM_WS=/root/swarm_ws

ENV FCU_URL=/dev/ttyTHS1:921600
ENV VO_TOPIC=/d2vins/odometry
ENV VO_IMU_TOPIC=/d2vins/imu_propagation
ENV DRONE_ID=1

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y git ros-${ROS_VERSION}-mavros ros-${ROS_VERSION}-mavros-extras ros-${ROS_VERSION}-mavros-msgs \
      vim wget screen libglib2.0-dev python3-termcolor python3-matplotlib
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
RUN chmod +x install_geographiclib_datasets.sh
RUN ./install_geographiclib_datasets.sh

#Install LCM
RUN   git clone https://github.com/lcm-proj/lcm && \
      cd lcm && \
      git checkout tags/v1.4.0 && \
      mkdir build && cd build && \
      cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF .. && \
      make -j$(nproc) install

#Build D2SLAM
RUN   mkdir -p ${SWARM_WS}/src/ && \
      cd ${SWARM_WS}/src/ && \
      git clone https://github.com/HKUST-Swarm/swarm_msgs.git -b D2SLAM && \
      git clone https://github.com/HKUST-Swarm/bspline
COPY ./ ${SWARM_WS}/src/
WORKDIR $SWARM_WS
SHELL ["/bin/bash", "-c"]
RUN   . "/opt/ros/${ROS_VERSION}/setup.sh" && \
      catkin_make -DCMAKE_BUILD_TYPE=Release
COPY ./entrypoint.sh /
COPY ./drone_position_control/launch/pos_control_param.yaml /

ENTRYPOINT ["/entrypoint.sh"]
