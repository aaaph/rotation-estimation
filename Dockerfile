FROM ros:kilted-ros-base

RUN apt-get update && apt-get install -y \
    ros-kilted-rmw-zenoh-cpp \
    && rm -rf /var/lib/apt/lists/*

ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
ENV ROS_DOMAIN_ID=0

RUN echo "source /opt/ros/kilted/setup.bash" >> ~/.bashrc
