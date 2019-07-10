FROM ros:kinetic-ros-core-xenial

# Make the IGVC workspace directory
RUN mkdir -p /igvc_ws/src

# Copy the src folder into the Docker igvc workspace
COPY igvc_ws/src /igvc_ws/src

# Navigate into the workspace
WORKDIR /igvc_ws

RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash; catkin_make'
