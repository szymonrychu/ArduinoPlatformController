#!/bin/bash
REMOTE_IP="$1"
if [ -z $REMOTE_IP ]; then
    echo "Remote IP missing!" 1>&2
    exit 1
fi
if ! cat /etc/hosts | grep "${REMOTE_IP} robot" > /dev/null; then
    echo "${REMOTE_IP} robot" | sudo tee -a /etc/hosts
fi
ssh-keyscan -t rsa ${REMOTE_IP} >> ~/.ssh/known_hosts

LOCAL_IP=$(hostname -I | awk '{print $1}')

if ! ssh -o PasswordAuthentication=no  -o BatchMode=yes ubuntu@${REMOTE_IP} exit &>/dev/null; then
    echo "Ensuring ssh keys are present on remote."
    if ! which sshpass > /dev/null; then
        echo "sshpass missing! install with 'sudo apt-get install sshpass -y'" 1>&2
        exit 1
    fi
    if [ -z $REMOTE_PASSWORD ]; then
        echo -n "Provide remote password: "
        read -s REMOTE_PASSWORD
    fi
    sshpass -p "$REMOTE_PASSWORD" scp ~/.ssh/id_rsa ubuntu@${REMOTE_IP}:/tmp/
    sshpass -p "$REMOTE_PASSWORD" scp ~/.ssh/id_rsa.pub ubuntu@${REMOTE_IP}:/tmp/
    sshpass -p "$REMOTE_PASSWORD" ssh ubuntu@${REMOTE_IP} \
        'bash -c "\
            mkdir -p ~/.ssh;\
            mv /tmp/id_rsa /tmp/id_rsa.pub ~/.ssh/;\
            chmod 0600 ~/.ssh/id_rsa;\
            cat ~/.ssh/id_rsa.pub >> ~/.ssh/authorized_keys"'
fi

if ! ssh ubuntu@${REMOTE_IP} 'ls ~/catkin_ws > /dev/null'; then
    echo "Ensuring ROS noetic is present on remote."
    ssh ubuntu@${REMOTE_IP} 'bash -c "\
        sudo apt-get update;\
        sudo apt-get install python3-pip build-essential git -y;\
        sudo pip3 install --upgrade pip;\
        cd /tmp/;\
        ssh-keyscan -t rsa github.com >> ~/.ssh/known_hosts;\
        rm -rf ArduinoPlatformController;\
        git clone git@github.com:szymonrychu/ArduinoPlatformController.git;\
        ./ArduinoPlatformController/install_ros.sh;\
        mv ArduinoPlatformController ~/catkin_ws/src/"'
fi

if ! ssh ubuntu@${REMOTE_IP} "cat /etc/hosts | grep '${REMOTE_IP} robot'"; then
    ssh ubuntu@${REMOTE_IP} "echo '${REMOTE_IP} robot' | sudo tee -a /etc/hosts"
fi

if ! ssh ubuntu@${REMOTE_IP} "cat /etc/hosts | grep '${LOCAL_IP} overlord'"; then
    ssh ubuntu@${REMOTE_IP} "echo '${LOCAL_IP} overlord' | sudo tee -a /etc/hosts"
fi

if ! ssh ubuntu@${REMOTE_IP} 'ls /etc/systemd/system/roscore.service'; then
    ssh ubuntu@${REMOTE_IP} 'bash -c "\
        cp ~/catkin_ws/src/ArduinoPlatformController/roscore.service /etc/systemd/system/roscore.service;\
        sudo systemctl enable roscore;\
        sudo systemctl start roscore"'
fi

ssh ubuntu@${REMOTE_IP} 'bash -c "\
    source /opt/ros/noetic/setup.bash;\
    cd ~/catkin_ws;\
    catkin_make -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic install"'
