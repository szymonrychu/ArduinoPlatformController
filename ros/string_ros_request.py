#!/usr/bin/env python3

import rospy
import sys
import argparse
import time

from std_msgs.msg import String


class FireForgetPublisher():
   def __init__(self, topic_n, data):
      self._data = String()
      self._data.data = data
      self._publisher = rospy.Publisher(topic_n, String, queue_size=10)

   def publish(self, *_args, **_kwargs):
      self._publisher.publish(self._data)
      time.sleep(1)
      rospy.signal_shutdown('')

   def start(self):
      rospy.init_node('tmp_node')
      rospy.Timer(rospy.Duration(0.1), callback=self.publish, oneshot=True)
      rospy.spin()

def main():
   parser = argparse.ArgumentParser()
   parser.add_argument('--topic', '-t', type=str, required=False, default='/robot_platform/raw_serial_request')
   parser.add_argument('--data', '-d', type=str, required=True)
   args = parser.parse_args()

   FireForgetPublisher(args.topic, args.data).start()

if __name__ == '__main__':
   main()



'''
source /opt/ros/noetic/setup.bash 
./ros/string_ros_request.py -d '{"move_duration":2,"motor1":{"angle":-0.0},"motor2":{"angle":-0.0},"motor3":{"angle":0.0},"motor4":{"angle":0.0}}'


./ros/string_ros_request.py -d '{"move_duration":2,"motor1":{"angle":-0.7853},"motor2":{"angle":-0.7853},"motor3":{"angle":0.7853},"motor4":{"angle":0.7853}}'

./ros/string_ros_request.py -d '{"move_duration":2,"motor1":{"angle":-0.7853}}'
./ros/string_ros_request.py -d '{"move_duration":2,"motor1":{"angle":-0.7853}}'
./ros/string_ros_request.py -d '{"move_duration":2,"motor2":{"angle":-0.7853}}'
./ros/string_ros_request.py -d '{"move_duration":2,"motor3":{"angle":-0.7853}}'
./ros/string_ros_request.py -d '{"move_duration":2,"motor4":{"angle":-0.7853}}'


~/catkin_ws/src/string_ros_request.py -d '{"motor1":{"angle":2.187251793041891},"motor2":{"angle":0.9543408605479022},"motor3":{"angle":-0.9543408605479022},"motor4":{"angle":-2.18725179304189},"move_duration":1.5000919909545711}'
./ros/string_ros_request.py -d '{"motor1":{"velocity":-0.043},"motor2":{"velocity":-0.043},"motor3":{"velocity":-0.043},"motor4":{"velocity":-0.043},"pan":{},"tilt":{},"move_duration":3.927}'

LT
~/catkin_ws/src/string_ros_request.py -d '{"move_duration":2,"motor1":{"angle":1.5707}'
PT
~/catkin_ws/src/string_ros_request.py -d '{"move_duration":2,"motor2":{"angle":1.5707}'
PP
~/catkin_ws/src/string_ros_request.py -d '{"move_duration":2,"motor3":{"angle":1.5707}'
LP
~/catkin_ws/src/string_ros_request.py -d '{"move_duration":2,"motor4":{"angle":1.5707}'

45
~/catkin_ws/src/string_ros_request.py -d '{"move_duration":2,"motor1":{"angle":-0.7853},"motor2":{"angle":-0.7853},"motor3":{"angle":0.7853},"motor4":{"angle":0.7853}}'
90
~/catkin_ws/src/string_ros_request.py -d  '{"move_duration":3,"motor1":{"angle":1.5707},"motor2":{"angle":1.5707},"motor3":{"angle":1.5707},"motor4":{"angle":1.5707}}'
-90
~/catkin_ws/src/string_ros_request.py -d '{"move_duration":3,"motor1":{"angle":-1.5707},"motor2":{"angle":-1.5707},"motor3":{"angle":-1.5707},"motor4":{"angle":-1.5707}}'


0
~/catkin_ws/src/string_ros_request.py -d '{"move_duration":3,"motor1":{"angle":0},"motor2":{"angle":0},"motor3":{"angle":0},"motor4":{"angle":0}'

motors stop
~/catkin_ws/src/string_ros_request.py -d '{"motor1":{"velocity":0},"motor2":{"velocity":0},"motor3":{"velocity":0},"motor4":{"velocity":0}}'

0.2 forward for 2s
~/catkin_ws/src/string_ros_request.py -d '{"move_duration":2,"motor1":{"velocity":0.2},"motor2":{"velocity":0.2},"motor3":{"velocity":0.2},"motor4":{"velocity":0.2}}'

0.2 forward for 1s
~/catkin_ws/src/string_ros_request.py -d '{"move_duration":2,"motor1":{"velocity":1.0},"motor2":{"velocity":1.0},"motor3":{"velocity":1.0},"motor4":{"velocity":1.0}}'

0.3 backward for 2s
~/catkin_ws/src/string_ros_request.py -d '{"move_duration":2,"motor1":{"velocity":-0.3},"motor2":{"velocity":-0.3},"motor3":{"velocity":-0.3},"motor4":{"velocity":-0.3}}'
'''