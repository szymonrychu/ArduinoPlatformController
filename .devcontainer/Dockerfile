FROM ros:noetic-ros-base-focal

RUN set -xe;\
    apt-get update;\
    apt-get install -y --no-install-recommends \
        cmake \
        curl \
        git \
        libfreenect-dev \
        python3-pip \
        python3-tf-conversions \
        python3-tf2-ros \
        ros-noetic-angles \
        ros-noetic-camera-info-manager \
        ros-noetic-diagnostic-updater \
        ros-noetic-robot-localization \
        ros-noetic-hector-geotiff-launch \
        ros-noetic-hector-localization \
        ros-noetic-hector-map-server \
        ros-noetic-hector-mapping \
        ros-noetic-rgbd-launch \
        ros-noetic-robot-navigation \
        ros-noetic-base-local-planner \
        ros-noetic-dwa-local-planner \
        ros-noetic-mpc-local-planner \
        ros-noetic-teb-local-planner \
        ros-noetic-urdf \
        ros-noetic-global-planner \
        ros-noetic-move-base \
        ros-noetic-xacro \
        ros-noetic-joy \
        ros-noetic-rviz \
        vim \
        libxcb-xinerama0 \
        libx11-xcb1 \
        libxcb-util1 \
        libglu1-mesa \
        libxrender1 \
        libxi6 \
        libxtst6 \
        libxrandr2 \
        libxss1 \
        libxcursor1 \
        libxcomposite1 \
        libxdamage1 \
        libxext6 \
        libsm6 \
        libice6 \
        x11-xserver-utils \
        ;\
    rm -rf /var/lib/apt/lists/*;\
    pip3 install platformio

RUN useradd -m -s /bin/bash -G sudo,dialout ros;\
    echo 'ros ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/ros;\
    mkdir -p /home/ros/catkin_ws/src;\
    mkdir -p /home/ros/arduino;\
    chown -R ros /opt/ros /home/ros

USER ros

RUN set -xe;\
    bash -c "\
        echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /home/ros/.bashrc;\
        source /opt/ros/${ROS_DISTRO}/setup.bash;\
        rosdep update"

RUN set -xe;\
    bash -c "\
        source /opt/ros/${ROS_DISTRO}/setup.bash;\
        echo 'source /home/ros/catkin_ws/devel/setup.bash' >> /home/ros/.bashrc;\
        echo 'export PYTHONPATH="$PYTHONPATH:/home/ros/catkin_ws/devel/lib/python3/dist-packages" >> /home/ros/.bashrc';\
        cd /home/ros/catkin_ws;\
        catkin_make"


WORKDIR /home/ros/catkin_ws
