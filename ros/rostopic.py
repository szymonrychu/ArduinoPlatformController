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
~/catkin_ws/src/ArduinoPlatformController/ros/rostopic.py /robot_platform/raw_serial_request \
   '{"message_type":"forward", "move_distance":0.25, "move_velocity": 0.5, "UUID":"a5ad3b48-71d1-11ee-b962-0242ac120002"}'
~/catkin_ws/src/ArduinoPlatformController/ros/rostopic.py /robot_platform/raw_serial_request \
   '{"message_type":"forward", "move_distance":-0.25, "move_velocity": 0.5, "UUID":"a5ad3b48-71d1-11ee-b962-0242ac120002"}'

~/catkin_ws/src/ArduinoPlatformController/ros/rostopic.py /robot_platform/raw_serial_request \
   '{"message_type":"turn", "turn_angle":0.7853, "turn_velocity": 0.5, "UUID":"a5ad3b48-71d1-11ee-b962-0242ac120002"}'
~/catkin_ws/src/ArduinoPlatformController/ros/rostopic.py /robot_platform/raw_serial_request \
   '{"message_type":"turn", "turn_angle":-0.7853, "turn_velocity": 0.5, "UUID":"a5ad3b48-71d1-11ee-b962-0242ac120002"}'

~/catkin_ws/src/ArduinoPlatformController/ros/rostopic.py /robot_platform/raw_serial_request \
   '{"message_type":"sequentional_move","moves":[{"message_type":"reset_queue"},{"message_type":"turn", "turn_angle":0.7853, "turn_velocity": 0.5, "UUID":"a5ad3b48-71d1-11ee-b962-0242ac120002"},{"message_type":"forward", "move_distance":0.25, "move_velocity": 0.5, "UUID":"2a0d6d86-71d2-11ee-b962-0242ac120002"}]}'
'''