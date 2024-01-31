#!/usr/bin/env python3

import rospy
import sys
import argparse
import time

import tf_conversions

from geometry_msgs.msg import PoseStamped, Quaternion


class FireForgetPublisher():
   def __init__(self, topic_n, x, y, yaw, frame_id):
      self._data = PoseStamped()
      self._data.header.frame_id = frame_id
      self._data.pose.position.x = x
      self._data.pose.position.y = y

      q = tf_conversions.transformations.quaternion_from_euler(0, 0, yaw)
      self._data.pose.orientation = Quaternion(*q)
      
      self._publisher = rospy.Publisher(topic_n, PoseStamped, queue_size=10)

   def publish(self, *_args, **_kwargs):
      self._data.header.stamp = rospy.Time.now()
      self._publisher.publish(self._data)
      time.sleep(1)
      rospy.signal_shutdown('')

   def start(self):
      rospy.init_node('tmp_node')
      rospy.Timer(rospy.Duration(0.1), callback=self.publish, oneshot=True)
      rospy.spin()

def main():
   parser = argparse.ArgumentParser()
   parser.add_argument('--topic', '-t', type=str, required=False, default='/move_base_simple/goal')
   parser.add_argument('-x', type=float, required=True)
   parser.add_argument('-y', type=float, required=True)
   parser.add_argument('--yaw', type=float, required=True)
   parser.add_argument('-f', '--frame-id', type=str, required=False, default='')
   args = parser.parse_args()
   
   FireForgetPublisher(args.topic, args.x, args.y, args.yaw, args.frame_id).start()

if __name__ == '__main__':
   main()


'''
source /opt/ros/noetic/setup.bash 

~/catkin_ws/src/goal_ros_request.py -x 1.0 -y 1.0 --yaw 0.7853
~/catkin_ws/src/goal_ros_request.py -x 1.0 -y -1.0 --yaw 0.7853

~/catkin_ws/src/goal_ros_request.py -x 0.0 -y 1.0 --yaw 0.7853
~/catkin_ws/src/goal_ros_request.py -x 0.0 -y -1.0 --yaw 0.7853

~/catkin_ws/src/goal_ros_request.py -x 1.0 -y 0.0 --yaw 0.7853

~/catkin_ws/src/goal_ros_request.py -x -1.0 -y 0.0 --yaw 0.7853

~/catkin_ws/src/goal_ros_request.py -x 0.0 -y 0.0 --yaw 0.0


'''