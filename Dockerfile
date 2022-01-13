FROM osrf/ros:melodic-desktop

RUN apt-get update && apt-get install wget -y
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

SHELL ["/bin/bash", "-c"] 
RUN "source /opt/ros/melodic/setup.bash"

RUN apt-get update \
 && apt-get install -y --no-install-recommends \
 gazebo9 \
 libgazebo9-dev \
 ros-melodic-gazebo-ros \
 ros-melodic-gazebo-ros-pkgs  \
 ros-melodic-gazebo-ros-control \
 python3-catkin-tools \
 python3-osrf-pycommon \
&& rm -rf /var/lib/apt/lists/*

# install sjtu-drone package
RUN mkdir -p /home/catkin_ws/src
WORKDIR "/home/catkin_ws/src"
RUN git clone https://github.com/tahsinkose/sjtu-drone.git
WORKDIR "/home/catkin_ws"
RUN catkin_make
RUN "source /home/catkin_ws/devel/setup.bash"

# install drone3d package
WORKDIR "/home/catkin_ws/src"
RUN git clone https://github.com/longfish/drone3d.git
WORKDIR "/home/catkin_ws"
RUN catkin_make

LABEL Name=drone3d Version=0.0.1
