#!/usr/bin/env python3

import rospy
import argparse
import time

import tf_conversions

from typing import Type
from geometry_msgs.msg import PoseStamped, Quaternion
from robot_platform.msg import PlatformRequest, ServoRequest, MotorRequest

import genpy


class FireForgetPublisher():
   def __init__(self, topic:str, type:Type, data:genpy.Message):
      rospy.init_node('tmp_node')
      rospy.Timer(rospy.Duration.from_sec(0.1), callback=self._publish, oneshot=True)
      rospy.spin()
      self._publisher = rospy.Publisher(topic, data_class=type)
      self.__data = data

   def _publish(self, *_args, **_kwargs):
      self._publisher.publish(self.__data)
      time.sleep(1)
      rospy.signal_shutdown('')


def main():
   parser = argparse.ArgumentParser()
   subparsers = parser.add_subparsers(help='ROS operation', dest='command')

   goal_parser = subparsers.add_parser("goal")
   goal_parser.add_argument('--topic', '-t', type=str, required=False, default='/move_base_simple/goal')
   goal_parser.add_argument('-x', type=float, required=True)
   goal_parser.add_argument('-y', type=float, required=True)
   goal_parser.add_argument('--yaw', type=float, required=True)
   goal_parser.add_argument('-f', '--frame-id', type=str, required=False, default='')

   raw_parser = subparsers.add_parser("raw")
   raw_parser.add_argument('--topic', '-t', type=str, required=False, default='/robot_platform/wheel_request')
   raw_parser.add_argument('--duration', '-d', type=float, required=True)
   raw_parser.add_argument('--motor1-velocity', '-m1v', type=float, required=False, default=None)
   raw_parser.add_argument('--motor2-velocity', '-m2v', type=float, required=False, default=None)
   raw_parser.add_argument('--motor3-velocity', '-m3v', type=float, required=False, default=None)
   raw_parser.add_argument('--motor4-velocity', '-m4v', type=float, required=False, default=None)
   raw_parser.add_argument('--servo1-angle', '-s1a', type=float, required=False, default=None)
   raw_parser.add_argument('--servo2-angle', '-s2a', type=float, required=False, default=None)
   raw_parser.add_argument('--servo3-angle', '-s3a', type=float, required=False, default=None)
   raw_parser.add_argument('--servo4-angle', '-s4a', type=float, required=False, default=None)
   raw_parser.add_argument('--pan-angle', '-pa', type=float, required=False, default=None)
   raw_parser.add_argument('--tilt-angle', '-ta', type=float, required=False, default=None)

   args = parser.parse_args()
   inputs = vars(args)
   
   if inputs['command'] == 'raw':
      request = PlatformRequest()
      request.duration = inputs['duration']
      if inputs['motor1_velocity']:
         request.motor1 = MotorRequest(velocity=inputs['motor1_velocity'])
      if inputs['motor2_velocity']:
         request.motor2 = MotorRequest(velocity=inputs['motor2_velocity'])
      if inputs['motor3_velocity']:
         request.motor3 = MotorRequest(velocity=inputs['motor3_velocity'])
      if inputs['motor4_velocity']:
         request.motor4 = MotorRequest(velocity=inputs['motor4_velocity'])
      
      if inputs['servo1_angle']:
         request.servo1 = ServoRequest(angle=inputs['servo1_angle'])
      if inputs['servo2_angle']:
         request.servo2 = ServoRequest(angle=inputs['servo2_angle'])
      if inputs['servo3_angle']:
         request.servo3 = ServoRequest(angle=inputs['servo3_angle'])
      if inputs['servo4_angle']:
         request.servo4 = ServoRequest(angle=inputs['servo4_angle'])
   
      if inputs['pan_angle']:
         request.pan = ServoRequest(angle=inputs['pan_angle'])
      if inputs['tilt_angle']:
         request.tilt = ServoRequest(angle=inputs['tilt_angle'])
      FireForgetPublisher(inputs['topic'], PlatformRequest, request)
   
   if inputs['command'] == 'goal':
      request = PoseStamped()
      request.header.frame_id = inputs['frame_id']
      request.header.stamp = rospy.Time.now()
      request.pose.position.x = inputs['x']
      request.pose.position.y = inputs['y']
      q = tf_conversions.transformations.quaternion_from_euler(0, 0, inputs['yaw'])
      request.pose.orientation = Quaternion(*q)
      FireForgetPublisher(inputs['topic'], PoseStamped, request)

if __name__ == '__main__':
   main()

