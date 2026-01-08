FROM ros:noetic-robot

SHELL ["/bin/bash", "-c"]
WORKDIR /
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws

RUN apt-get update && apt-get install git -y
RUN cd src && git clone https://github.com/foxglove/ros-foxglove-bridge.git
RUN . /ros_entrypoint.sh && \
    apt-get update && \
    ( rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y -r || true )
RUN . /ros_entrypoint.sh && catkin_make
COPY ./ros-connector.launch .
COPY ./launch_ros_connector.sh /
