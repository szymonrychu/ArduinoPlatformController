#!/usr/bin/env python3

import rospy
import sys

from std_msgs.msg import String


class FireForgetPublisher():
    def __init__(self, topic_n, data):
        self._data = String()
        self._data.data = data
        self._publisher = rospy.Publisher(topic_n, String, queue_size=10)

    def publish(self, *_args, **_kwargs):
        self._publisher.publish(self._data)
        rospy.signal_shutdown('')

    def start(self):
        rospy.init_node('tmp_node')
        rospy.Timer(rospy.Duration(0.1), callback=self.publish, oneshot=True)
        rospy.spin()

FireForgetPublisher(sys.argv[1], sys.argv[2]).start()

'''
source /opt/ros/noetic/setup.bash 

~/catkin_ws/src/rostopic.py /robot_platform/raw_serial_request '{"move_duration":2,"motor1":{"angle":-0.7853},"motor2":{"angle":-0.7853},"motor3":{"angle":0.7853},"motor4":{"angle":0.7853}}'

~/catkin_ws/src/rostopic.py /robot_platform/raw_serial_request '{"move_duration":2,"motor1":{"angle":-0.0},"motor2":{"angle":-0.0},"motor3":{"angle":0.0},"motor4":{"angle":0.0}}'


45
~/catkin_ws/src/ArduinoPlatformController/ros/rostopic.py /robot_platform/raw_serial_request \
   '{"move_duration":2,"motor1":{"angle":-0.7853},"motor2":{"angle":-0.7853},"motor3":{"angle":0.7853},"motor4":{"angle":0.7853}}'
90
~/catkin_ws/src/ArduinoPlatformController/ros/rostopic.py /robot_platform/raw_serial_request \
   '{"move_duration":3,"motor1":{"angle":1.5707},"motor2":{"angle":1.5707},"motor3":{"angle":1.5707},"motor4":{"angle":1.5707}}'
-90
~/catkin_ws/src/ArduinoPlatformController/ros/rostopic.py /robot_platform/raw_serial_request \
   '{"move_duration":3,"motor1":{"angle":-1.5707},"motor2":{"angle":-1.5707},"motor3":{"angle":-1.5707},"motor4":{"angle":-1.5707}}'
-90 1 motor
~/catkin_ws/src/ArduinoPlatformController/ros/rostopic.py /robot_platform/raw_serial_request \
   '{"move_duration":3,"motor1":{"angle":0.7}}'
0
~/catkin_ws/src/ArduinoPlatformController/ros/rostopic.py /robot_platform/raw_serial_request \
   '{"move_duration":3,"motor1":{"angle":0},"motor2":{"angle":0},"motor3":{"angle":0},"motor4":{"angle":0}'

motors stop
~/catkin_ws/src/ArduinoPlatformController/ros/rostopic.py /robot_platform/raw_serial_request \
   '{"motor1":{"velocity":0},"motor2":{"velocity":0},"motor3":{"velocity":0},"motor4":{"velocity":0}}'

0.2 forward for 2s
~/catkin_ws/src/ArduinoPlatformController/ros/rostopic.py /robot_platform/raw_serial_request \
   '{"move_duration":2,"motor1":{"velocity":0.2},"motor2":{"velocity":0.2},"motor3":{"velocity":0.2},"motor4":{"velocity":0.2}}'

0.2 forward for 1s
~/catkin_ws/src/ArduinoPlatformController/ros/rostopic.py /robot_platform/raw_serial_request \
   '{"move_duration":2,"motor1":{"velocity":1.0},"motor2":{"velocity":1.0},"motor3":{"velocity":1.0},"motor4":{"velocity":1.0}}'

0.3 backward for 2s
~/catkin_ws/src/ArduinoPlatformController/ros/rostopic.py /robot_platform/raw_serial_request \
   '{"move_duration":2,"motor1":{"velocity":-0.3},"motor2":{"velocity":-0.3},"motor3":{"velocity":-0.3},"motor4":{"velocity":-0.3}}'
'''