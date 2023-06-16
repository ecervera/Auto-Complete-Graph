FROM osrf/ros:noetic-desktop-full

RUN apt-get update && apt-get install -y git python3-vcstool \
 && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /catkin_ws/src

RUN cd /catkin_ws/src \
 && git clone https://github.com/ecervera/Auto-Complete-Graph.git 

RUN cd /catkin_ws \
 && vcs import src < src/Auto-Complete-Graph/rosinstall \
 && vcs pull src

RUN cd /catkin_ws \
 && apt-get update \
 && rosdep install --from-paths src --ignore-src -r -y \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*

SHELL ["/bin/bash", "-c"]

RUN cd /catkin_ws \
 && source /opt/ros/${ROS_DISTRO}/setup.bash \
 && catkin_make_isolated

