#!/usr/bin/env python3

import rospy
import argparse
import time
import math

import tf_conversions

from typing import Type
from geometry_msgs.msg import PoseStamped, Quaternion
from robot_platform.msg import PlatformRequest, MotorRequest, ServoRequest
from std_msgs.msg import Float32

import genpy

class FireForgetPublisher():
   def __init__(self, topic:str, typ:Type, data:genpy.Message):
      self._publisher = rospy.Publisher(topic, typ, queue_size=1)
      self.__data = data
      rospy.Timer(rospy.Duration.from_sec(0.1), callback=self._publish, oneshot=True)
      rospy.spin()

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
   
   rospy.init_node('tmp_node')

   if inputs['command'] == 'raw':
      request = PlatformRequest()
      request.duration = inputs['duration']
      if inputs['motor1_velocity']:
         request.motor1 = MotorRequest(velocity=inputs['motor1_velocity'], defined=True)
      if inputs['motor2_velocity']:
         request.motor2 = MotorRequest(velocity=inputs['motor2_velocity'], defined=True)
      if inputs['motor3_velocity']:
         request.motor3 = MotorRequest(velocity=inputs['motor3_velocity'], defined=True)
      if inputs['motor4_velocity']:
         request.motor4 = MotorRequest(velocity=inputs['motor3_velocity'], defined=True)
      
      if inputs['servo1_angle']:
         request.servo1 = ServoRequest(angle=math.radians(inputs['servo1_angle']), defined=True)
      if inputs['servo2_angle']:
         request.servo2 = ServoRequest(angle=math.radians(inputs['servo2_angle']), defined=True)
      if inputs['servo3_angle']:
         request.servo3 = ServoRequest(angle=math.radians(inputs['servo3_angle']), defined=True)
      if inputs['servo4_angle']:
         request.servo4 = ServoRequest(angle=math.radians(inputs['servo4_angle']), defined=True)
   
      if inputs['pan_angle']:
         request.pan = ServoRequest(angle=math.radians(inputs['pan_angle']), defined=True)
      if inputs['tilt_angle']:
         request.tilt = ServoRequest(angle=math.radians(inputs['tilt_angle']), defined=True)
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

