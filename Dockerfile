FROM ubuntu:bionic as base

ENV DEBIAN_FRONTEND=noninteractive

RUN set -xe;\
    apt-get update -q;\
    apt-get install -y --no-install-recommends sudo gnupg2 bash git vim;\
    useradd -m -s /bin/bash -G sudo,dialout -d /home/ros ros;\
    chown -R ros. /home/ros;\
    echo "ros ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/ros

RUN set -xe;\
    echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list;\
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654;\
    apt-get update -q;\
    apt-get install -y --no-install-recommends \
        ros-melodic-robot \
        python-rosdep \
        python-setuptools \
        python-pip \
        python-wheel \
        cython \
        python-rosinstall \
        python-rosinstall-generator \
        python-wstool \
        build-essential;\
    rosdep init;\
    chown -R ros. /opt/ros/melodic

USER ros

RUN set -xe;\
    echo "source /opt/ros/melodic/setup.bash" >> $HOME/.bashrc;\
    rosdep update;\
    bash -c "\
        source /opt/ros/melodic/setup.bash;\
        mkdir -p $HOME/catkin_ws/src;\
        cd $HOME/catkin_ws;\
        catkin_make;\
    "

FROM base as robot

COPY ./ros_omni_platform /home/ros/catkin_ws/src/omni_platform

WORKDIR /home/ros

RUN set -xe;\
    bash -c "\
        source /opt/ros/melodic/setup.bash;\
        source $HOME/catkin_ws/devel/setup.bash;\
        cd $HOME/catkin_ws;\
        pip install -r /home/ros/catkin_ws/src/omni_platform/requirements.txt;\
        catkin_make -DCMAKE_INSTALL_PREFIX=/opt/ros/melodic install;\
    "

CMD ["/home/ros/catkin_ws/src/omni_platform/run.sh"]