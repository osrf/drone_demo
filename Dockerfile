FROM osrf/ros:melodic-desktop-full

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

RUN /bin/sh -c 'echo ". /opt/ros/melodic/setup.bash" >> ~/.bashrc' \
 && /bin/sh -c 'echo ". /usr/share/gazebo/setup.sh" >> ~/.bashrc'

# Needed until upcoming sync with mavlink 2018.12.12
# RUN sed -i 's|/ros/|/ros-shadow-fixed/|' /etc/apt/sources.list.d/ros-latest.list

# Add ROS2 sources
RUN /bin/sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu bionic main" > /etc/apt/sources.list.d/ros2-latest.list'
# RUN curl http://repo.ros2.org/repos.key | apt-key add -

# Optimizing for build time preinstalling dependencies
RUN apt-get update \
 && apt-get dist-upgrade -y \ 
 && apt-get clean
RUN apt-get update \
 && apt-get install -y \
    ros-melodic-mavros \
    unzip \
    python-toml \
    speech-dispatcher \
    python-markupsafe \
    libasound2-dev libdbus-1-dev libpulse-dev libpulse-mainloop-glib0 libsdl2-2.0-0 libsndio-dev libsndio6.1 libxv-dev x11proto-video-dev \
    protobuf-compiler \
    python-jinja2 libsdl2-dev \
    python3-catkin-pkg-modules \
    python3-rospkg-modules \
    python3-tk \
 && apt-get clean

# optional dependency for qgc
RUN apt-get update \
 && apt-get install -y \
    speech-dispatcher \
    libimage-exiftool-perl \
    gstreamer1.0-libav \
 && apt-get clean

# optional dependency for camera streaming
RUN apt-get update \
 && apt-get install -y \
    gstreamer1.0-alsa \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-ugly \
 && apt-get clean

 # Install geographic lib dataset, it should be in a post install hook, but isn't 
 # https://github.com/mavlink/mavros/issues/1005
RUN bash /opt/ros/melodic/lib/mavros/install_geographiclib_datasets.sh

# Preinstall crystal for convenience
RUN apt-get update \
 && apt-get install -y \
    ros-crystal-desktop \
 && apt-get clean

 # Serial demo preinstalls
RUN apt-get update \
 && apt-get install -y \
    socat \
    byobu \
    python3-colcon* \
 && apt-get clean


RUN mkdir /workspace/drone_demo/src -p
WORKDIR /workspace/drone_demo/src
RUN git clone https://github.com/osrf/drone_demo.git -b master
RUN git clone https://github.com/tfoote/sitl_gazebo.git -b xacro_merge --recursive
RUN git clone https://github.com/osrf/uav_testing.git -b master

RUN mkdir /workspace/drone_demo_ros2/src -p
WORKDIR /workspace/drone_demo_ros2/src
RUN git clone https://github.com/osrf/ros2_serial_example.git

WORKDIR /workspace/drone_demo

# Make sure everything is up to date before building from source
RUN apt-get update \
 && apt-get dist-upgrade -y \ 
 && apt-get clean

WORKDIR /workspace/drone_demo_ros2
RUN . /opt/ros/crystal/setup.sh && rosdep update && rosdep install --from-path src -iy
RUN . /opt/ros/crystal/setup.sh && colcon build

WORKDIR /workspace/drone_demo


# Get fastrps tarball for fastrtpsgen binary
RUN wget https://www.eprosima.com/index.php/component/ars/repository/eprosima-fast-rtps/eprosima-fast-rtps-1-7-2/eprosima_fastrtps-1-7-2-linux-tar-gz?format=raw -O /tmp/fastrtps-1.7.2.tar.gz \
 && tar -xf /tmp/fastrtps-1.7.2.tar.gz

 # Micrortps demo dependencies
 # TODO(add as declared dependencies)
RUN apt-get update \
 && apt-get install -y \
    openjdk-8-jdk \
    rsync \
 && apt-get clean

ENV FASTRTPSGEN_DIR /workspace/drone_demo/eProsima_FastRTPS-1.7.2-Linux/bin

RUN . /opt/ros/melodic/setup.sh && rosdep update && rosdep install --from-path src -iy
RUN . /opt/ros/melodic/setup.sh && catkin config --install
RUN . /opt/ros/melodic/setup.sh && catkin build --verbose
# temporary partial rebuild for faster iteration touch this line to force a pull and rebuild
RUN cd /workspace/drone_demo/src/drone_demo && git pull
RUN . /opt/ros/melodic/setup.sh && catkin build --verbose

COPY entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD roslaunch sitl_launcher demo.launch gui:=false
