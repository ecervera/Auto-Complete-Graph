FROM osrf/ros:melodic-desktop-full

RUN apt-get update && apt-get install -y git python3-vcstool python3-catkin-tools \
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

RUN cd /catkin_ws/src/libraries/BetterGraph \
  && mkdir -p build && cd build \
  && cmake .. && make install

RUN cd /catkin_ws/src/libraries/VoDiGrEx \
  && mkdir -p build && cd build \
  && cmake .. && make install

RUN cd /catkin_ws/src/libraries/g2o \
  && mkdir -p build && cd build \
  && cmake .. && make install

SHELL ["/bin/bash", "-c"]

RUN cd /catkin_ws \
 && source /opt/ros/${ROS_DISTRO}/setup.bash \
 && catkin build

