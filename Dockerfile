FROM osrf/ros2:devel

# Tools I find useful during development
RUN apt-get update \
 && apt-get install -y \
    build-essential \
    lsb-release \
    python-catkin-tools \
    python-rosinstall \
    sudo \
    wget \
    ros-eloquent-gazebo-dev \
    ros-eloquent-gazebo-msgs \
    ros-eloquent-gazebo-ros \
    ros-eloquent-gazebo-plugins \
    ros-eloquent-gazebo-ros-pkgs \
    ros-eloquent-xacro \
    ros-eloquent-desktop \
 && apt-get clean

# Micrortps demo dependencies
# TODO(add as declared dependencies)
RUN apt-get update \
&& apt-get install -y \
  python-future \
  python3-future \
  python-lxml \
  python3-jinja2 -y \
  openjdk-8-jdk \
  rsync \
  python-empy \
  python-toml \
  python-numpy \
  python-catkin-pkg \
  python3-tk \
  libpulse-mainloop-glib0 \
  pulseaudio \
  && apt-get clean \
  && pip3 install catkin-pkg empy toml numpy tk future

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

# Make sure everything is up to date before building from source
RUN apt-get update \
  && apt-get dist-upgrade -y \
  && apt-get clean

RUN /bin/sh -c 'echo ". /opt/ros/eloquent/setup.bash" >> ~/.bashrc' \
  && /bin/sh -c 'echo ". /usr/share/gazebo/setup.sh" >> ~/.bashrc'

RUN mkdir  /workspace/drone_demo_ros2/src -p
WORKDIR /workspace/drone_demo_ros2/src
RUN git clone https://github.com/ahcorde/drone_demo.git -b ahcorde/ros2
RUN git clone https://github.com/osrf/sitl_gazebo -b ahcorde/urdf_models --recursive
RUN git clone https://github.com/osrf/uav_testing.git -b ros2
RUN git clone https://github.com/osrf/rviz_aerial_plugins.git
RUN git clone https://github.com/osrf/ros2_serial_example.git
RUN git clone https://github.com/PX4/px4_msgs.git

RUN git config --global  user.name "someone" && git config --global user.email "someone@someplace.com"
RUN cd /workspace/drone_demo_ros2/src/sitl_gazebo \
 && git fetch && git pull \
 && git cherry-pick f6708023fbc7466147de6c87ef7a3dad8c0aa19a e790b1ab0b44a14b573a2f1b4d45e0fb791a8f7f f9da71d926210398511c07109ed12681e5a1f7a7 322c22c162872a00b7bff9d7203e97dd11e06dc5 0a1660045afcd31e3529f469357a7db7c2736856 52993ec5904e79553744a4eb89bd654f70dd21c6

RUN apt-get update  && apt-get install python-jinja2 -y && apt-get clean

RUN git clone https://github.com/mavlink/mavlink -b 1.0.12 --recursive && \
    git clone https://github.com/mavlink/mavlink-gbp-release && \
    mv mavlink-gbp-release/patch/* mavlink

COPY files/mavlink/package.xml /workspace/drone_demo_ros2/src/mavlink/package.xml

WORKDIR /workspace

# Get fastrps tarball for fastrtpsgen binary
RUN wget https://www.eprosima.com/index.php/component/ars/repository/eprosima-fast-rtps/eprosima-fast-rtps-1-7-2/eprosima_fastrtps-1-7-2-linux-tar-gz?format=raw -O /tmp/fastrtps-1.7.2.tar.gz \
 && tar -xf /tmp/fastrtps-1.7.2.tar.gz

ENV FASTRTPSGEN_DIR /workspace/eProsima_FastRTPS-1.7.2-Linux/bin

WORKDIR /workspace/drone_demo_ros2

RUN . /opt/ros/eloquent/setup.sh && colcon build --merge-install --packages-skip ros2_serial_example --cmake-args -DBUILD_TESTING=False
RUN . /workspace/drone_demo_ros2/install/setup.sh && colcon build --merge-install --packages-select ros2_serial_example --cmake-args -DBUILD_TESTING=False -DROS2_SERIAL_PKGS="px4_msgs"

COPY entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# optional dependency for camera streaming
RUN apt-get update \
 && apt-get install -y \
    python3-contextlib2 \
 && apt-get clean

CMD ros2 launch sitl_launcher demo.launch.py use_rviz:=true use_qgroundcontrol:=true gui:=true verbose:=true sitl_world:=yosemite
