FROM osrf/ros:kinetic-desktop-full

# Tools I find useful during development
RUN apt-get update \
 && apt-get install -y \
    build-essential \
    lsb-release \
    python-catkin-tools \
    python-rosinstall \
    sudo \
    wget \
 && apt-get clean

RUN rosdep update

RUN /bin/sh -c 'echo ". /opt/ros/kinetic/setup.bash" >> ~/.bashrc' \
 && /bin/sh -c 'echo ". /usr/share/gazebo/setup.sh" >> ~/.bashrc'

# Needed until upcoming sync with mavlink 2018.12.12
RUN sed -i 's|/ros/|/ros-shadow-fixed/|' /etc/apt/sources.list.d/ros-latest.list

# Optimizing for build time preinstalling dependencies
RUN apt-get update \
 && apt-get dist-upgrade -y \ 
 && apt-get clean
RUN apt-get update \
 && apt-get install -y \
    ros-kinetic-mavros \
    unzip \
    python-toml \
    speech-dispatcher \
 && apt-get clean

# optional dependency for qgc 
RUN apt-get update \
 && apt-get install -y \
    speech-dispatcher \
 && apt-get clean


 # Install geographic lib dataset, it should be in a post install hook, but isn't 
 # https://github.com/mavlink/mavros/issues/1005
RUN sh /opt/ros/kinetic/lib/mavros/install_geographiclib_datasets.sh

RUN mkdir /workspace/drone_demo/src     -p
WORKDIR /workspace/drone_demo/src
RUN git clone https://github.com/osrf/drone_demo.git -b updates
RUN git clone https://github.com/PX4/sitl_gazebo.git --recursive

WORKDIR /workspace/drone_demo

# Make sure everything is up to date before building from source
RUN apt-get update \
 && apt-get dist-upgrade -y \ 
 && apt-get clean
RUN . /opt/ros/kinetic/setup.sh && rosdep update && rosdep install --from-path src -iy
RUN . /opt/ros/kinetic/setup.sh && catkin config --install
RUN . /opt/ros/kinetic/setup.sh && catkin build
COPY entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD roslaunch sitl_launcher demo.launch gui:=false