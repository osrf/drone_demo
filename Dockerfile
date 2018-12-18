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

# Optimizing for build time preinstalling dependencies
RUN apt-get update \
 && apt-get dist-upgrade -y \ 
 && apt-get clean
RUN apt-get update \
 && apt-get install -y \
    ros-kinetic-mavros \
    unzip \
    python-toml \
 && apt-get clean

# workspace development helpers
RUN apt-get update \
 && apt-get install -y \
    byobu \
    emacs \
 && apt-get clean

RUN mkdir /workspace/drone_demo/src     -p
WORKDIR /workspace/drone_demo/src
RUN git clone https://github.com/osrf/drone_demo.git -b updates
RUN git clone https://github.com/PX4/sitl_gazebo.git --recursive
# Pin sitl_gazebo until the matching change is out in debian packages. See discussion https://github.com/PX4/sitl_gazebo/pull/250
RUN cd sitl_gazebo && git checkout b05b6d5735068b7cf85bb9b529f5fa1318e1810a
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