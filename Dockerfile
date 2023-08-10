FROM ros:noetic-ros-base-focal

env DEBIAN_FRONTEND=noninteractive

COPY ./ros_entrypoint.sh /ros_entrypoint.sh 

RUN set -xe;\
    apt-get update;\
    apt-get install -y \
        git \
        vim \
        curl \
        libfreenect-dev \
        python3-pip \
        python3-tf2-ros \
        python3-tf-conversions \
        ros-noetic-xacro \
        ros-noetic-camera-info-manager \
        ros-noetic-diagnostic-updater \
        ;\
    rm -rf /var/lib/apt/lists/*;\
    useradd -m -s /bin/bash -G sudo,dialout ros;\
    echo 'ros ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/ros;\
    mkdir -p /home/ros/catkin_ws/src;\
    mkdir -p /home/ros/arduino;\
    chown -R ros /opt/ros;\
    chmod +x /ros_entrypoint.sh

USER ros

RUN set -xe;\
    bash -c "\
        source /opt/ros/${ROS_DISTRO}/setup.bash;\
        rosdep update"

COPY ./ros/ /home/ros/catkin_ws/src/

RUN set -xe;\
    bash -c "\
        source /opt/ros/${ROS_DISTRO}/setup.bash;\
        cd /home/ros/catkin_ws;\
        sudo chown -R ros .;\
        find ./src/ -maxdepth 2 -name requirements.txt -exec pip3 install -r {} \; ;\
        catkin_make -DCMAKE_INSTALL_PREFIX=/opt/ros/${ROS_DISTRO} install"

# RUN curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash - && sudo apt-get install -y nodejs;\
#     cd /home/ros/build/catkin_ws;\
#     git clone https://github.com/yxzhan/rvizweb.git src/rvizweb -b 0.1.0;\
#     rosdep install --from-paths src --ignore-src -r -y;\
#     bash -c "\
#         source /opt/ros/${ROS_DISTRO}/setup.bash;\
#         catkin_make install"

ENTRYPOINT [ "/ros_entrypoint.sh" ]

