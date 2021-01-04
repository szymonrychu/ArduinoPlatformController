#!/bin/bash
REMOTE_IP="$1"
if [ -z $REMOTE_IP ]; then
    echo "Remote IP missing!" 1>&2
    exit 1
fi

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

