FROM osrf/ros:melodic-desktop

RUN apt-get update && apt-get install wget -y
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

SHELL ["/bin/bash", "-c"]

RUN echo "source /opt/ros/melodic/setup.bash" >> /root/.bashrc
RUN bash -c "source /root/.bashrc"

RUN apt-get update \
 && apt-get install -y --no-install-recommends \
 psmisc \
 gazebo9 \
 libgazebo9-dev \
 ros-melodic-gazebo-ros \
 ros-melodic-gazebo-ros-pkgs  \
 ros-melodic-gazebo-ros-control \
 python3-catkin-tools \
 python3-osrf-pycommon \
&& rm -rf /var/lib/apt/lists/*

# create a workspace
RUN mkdir -p /home/catkin_ws/src
WORKDIR "/home/catkin_ws"

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
    
LABEL Name=drone3d Version=0.0.1
