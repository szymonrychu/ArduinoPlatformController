#!/usr/bin/env python3
from .platform_math import PlatformMath

import signal
import rospy
import tf_conversions
from geometry_msgs.msg import Twist, Vector3, PoseStamped, TransformStamped
from std_msgs.msg import String
import logging
from copy import copy
from threading import Lock

class Wheel():

    def __init__(self, _id, input_topic, output_topic, data_handler):
        self._handler = data_handler
        self._id = _id
        self.publisher = rospy.Publisher(output_topic, Vector3, queue_size=10)
        rospy.Subscriber(input_topic, TransformStamped, self.__callback)
        self._lock = Lock()
        self._fresh = False
    
    def __callback(self, data):
        with self._lock:
            self._fresh = True
            self._handler(self._id+1, data)

    @property
    def fresh(self):
        return self._lock
    
    def done()
        with self._lock:
            self._fresh = False
        

class Platform(PlatformMath):

    def __init__(self, wheel_input_topics, wheel_output_topics, tf2_base_link, tf2_output):
        self._wheels = []
        self._wheel_transforms = []
        for _id in range(Platform.WHEEL_NUM):
            self._wheels.append(Wheel(_id, wheel_input_topics[_id], wheel_output_topics[_id], self.wheel_output_hander))
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self._goal_callback)

    def wheel_output_hander(self, _id, data):


        rospy.loginfo(f"Received update from {_id}")
        pass

    def _goal_callback(self, data):
        dx = data.pose.position.x - self._current_pose.position.x
        dy = data.pose.position.y - self._current_pose.position.y
        dz = data.pose.position.z - self._current_pose.position.z

        r, p, y = tf_conversions.transformations.euler_from_quaternion([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])

        angle = math.atan2(dy, dx)
        distance = math.sqrt(dx*dx + dy*dy)
        rospy.loginfo(f"a/d:{angle}/{distance}")

        self.turn_in_place_and_move(angle, distance) #, 2000, 3000)
    


    def turn_and_move(self, distance, moving_time, angle, turning_time):
        distances = []
        angles = []
        for wheel_angle, wheel_distance in self.compute_angles_distances(angle, distance):
            angles.append(wheel_angle)
            distances.append(wheel_distance)
        
        if turning_time > 0:
            for wheel_id, wheel_angle in enumerate(angles):
                move_command = self.move_command(wheel_angle, 0, turning_time)
                rospy.loginfo("{}: {}".format(str(wheel_id+1), move_command))
                self.__threads[wheel_id].write_data(move_command)
            time.sleep(turning_time/1000.0)

        for wheel_id, (wheel_angle, wheel_distance) in enumerate(zip(angles, distances)):
            move_command = self.move_command(wheel_angle, wheel_distance, moving_time)
            rospy.loginfo("{}: {}".format(str(wheel_id+1), move_command))
            self.__threads[wheel_id].write_data(move_command)
        time.sleep(moving_time/1000.0)

    def turn_in_place(self, angle, moving_time):
        angles = list(self.get_in_place_angles())
        distances = list(self.get_in_place_angle2distance(angle))

        for wheel_id, wheel_angle in enumerate(angles):
            move_command = self.move_command(wheel_angle, 0.0, 2000)
            rospy.loginfo("{}: {}".format(str(wheel_id+1), move_command))
            self.__threads[wheel_id].write_data(move_command)
        time.sleep(2)
        for wheel_id, (wheel_angle, wheel_distance) in enumerate(zip(angles, distances)):
            move_command = self.move_command(wheel_angle, wheel_distance, moving_time)
            rospy.loginfo("{}: {}".format(str(wheel_id+1), move_command))
            self.__threads[wheel_id].write_data(move_command)
        time.sleep(moving_time/1000.0)

    def turn_in_place_and_move(self, angle, distance, turning_time=None, moving_time=None):
        if not turning_time:
            turning_time = abs(angle * 2000.0)
        if not moving_time:
            moving_time = abs(distance * 7000.0)
        self.turn_in_place(angle, turning_time)
        for wheel_id in range(PlatformMath.WHEEL_NUM):
            move_command = self.move_command(0.0, 0.0, 5)
            rospy.loginfo("{}: {}".format(str(wheel_id+1), move_command))
            self.__threads[wheel_id].write_data(move_command)
        time.sleep(1)
        for wheel_id in range(PlatformMath.WHEEL_NUM):
            move_command = self.move_command(0.0, distance, moving_time)
            rospy.loginfo("{}: {}".format(str(wheel_id+1), move_command))
            self.__threads[wheel_id].write_data(move_command)
        time.sleep(moving_time/1000.0)



def main():
    rospy.init_node('platform')
    wheel_input_topics = []
    wheel_output_topics = []
    for _id in range(PlatformMath.WHEEL_NUM):
        wheel_input_topics.append(rospy.get_param(f"~wheel{_id+1}_output_topic"))
        wheel_output_topics.append(rospy.get_param(f"~wheel{_id+1}_input_topic"))
    
    tf2_base_link = rospy.get_param("~tf2_base_link")
    tf2_output = rospy.get_param("~tf2_output")

    platform = Platform(wheel_input_topics, wheel_output_topics, tf2_base_link, tf2_output)
    rospy.spin()
    # signal.signal(signal.SIGINT, op.stop)
    # op.start()