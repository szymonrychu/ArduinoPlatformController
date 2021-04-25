#!/usr/bin/env python3
from .platform_math import PlatformMath

import signal
import rospy
import tf_conversions
import tf2_ros
from geometry_msgs.msg import Twist, Vector3, PoseStamped, TransformStamped
from std_msgs.msg import String
import logging
from copy import copy
from threading import Lock
import math
import time

class Meta():

    def __init__(self):
        self._tf2_base_link = ''
        self._tf2_link = ''

    def xyzRPY2TransformStamped(self, x, y, z, R, P, Y):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self._tf2_base_link
        t.child_frame_id = self._tf2_link
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        q = tf_conversions.transformations.quaternion_from_euler(R, P, Y)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        return t


class Wheel(Meta):

    def __init__(self, _id, input_topic, output_topic, translation, base_link, link, data_handler):
        self._handler = data_handler
        self._id = _id
        self._static_translation = translation
        self._tf2_link = link
        self._tf2_base_link = base_link
        self.publisher = rospy.Publisher(output_topic, Vector3, queue_size=10)
        rospy.Subscriber(input_topic, TransformStamped, self.__callback)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster()
    
    def __callback(self, data):
        self._handler(self._id+1, data)

    def send_transform(self):
        self._tf_broadcaster.sendTransform(self._static_translation)

    def send_command(self, distance, angle, time):
        v = Vector3()
        v.x = distance
        v.y = angle
        v.z = time
        self.publisher.publish(v)

class Platform(PlatformMath, Meta):

    def __init__(self, wheel_input_topics, wheel_output_topics, wheel_translations, wheel_base_links, wheel_links):
        self._wheels = []
        self._current_pose = PoseStamped()
        self._wheel_transforms = [None] * Platform.WHEEL_NUM
        self._tf2_base_link = "/map"
        self._tf2_link = "/base_link"
        self._lock = Lock()
        for _id in range(Platform.WHEEL_NUM):
            self._wheels.append(Wheel(_id, wheel_input_topics[_id], wheel_output_topics[_id], wheel_translations[_id], wheel_base_links[_id], wheel_links[_id], self.wheel_output_hander))
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self._goal_callback)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster()

    def wheel_output_hander(self, _id, data):
        with self._lock:
            self._wheel_transforms[_id-1] = data
            rospy.loginfo(f"{_id}: {data.transform.translation.x}:{data.transform.translation.y}")
            if all(self._wheel_transforms):
                x = sum([self._wheel_transforms[c].transform.translation.x for c in range(Platform.WHEEL_NUM)])/float(Platform.WHEEL_NUM)
                y = sum([self._wheel_transforms[c].transform.translation.y for c in range(Platform.WHEEL_NUM)])/float(Platform.WHEEL_NUM)

                front_x = (self._wheel_transforms[0].transform.translation.x + self._wheel_transforms[1].transform.translation.x)/2
                front_y = (self._wheel_transforms[0].transform.translation.y + self._wheel_transforms[1].transform.translation.y)/2
                Y = math.atan2(front_y-y, front_x-x)

                self._current_pose.pose.position.x = x
                self._current_pose.pose.position.y = y

                rospy.loginfo(f"Robot position: {x}, {y}, 0.0, 0.0, 0.0, {Y}")
                self._tf_broadcaster.sendTransform(self.xyzRPY2TransformStamped(x, y, 0, 0, 0, Y))
                for wheel in self._wheels:
                    wheel.send_transform()
                self._wheel_transforms = [None] * Platform.WHEEL_NUM

    def _goal_callback(self, data):
        dx = data.pose.position.x - self._current_pose.pose.position.x
        dy = data.pose.position.y - self._current_pose.pose.position.y
        dz = data.pose.position.z - self._current_pose.pose.position.z

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
            for _id, wheel_angle in enumerate(angles):
                self._wheels[_id].send_command(wheel_angle, 0, turning_time)
            time.sleep(turning_time/1000.0)

        for _id, (wheel_angle, wheel_distance) in enumerate(zip(angles, distances)):
            self._wheels[_id].send_command(wheel_angle, wheel_distance, moving_time)
        time.sleep(moving_time/1000.0)

    def turn_in_place(self, angle, moving_time):
        angles = list(self.get_in_place_angles())
        distances = list(self.get_in_place_angle2distance(angle))

        for _id, wheel_angle in enumerate(angles):
            self._wheels[_id].send_command(wheel_angle, 0.0, 2000)

        time.sleep(2)
        for _id, (wheel_angle, wheel_distance) in enumerate(zip(angles, distances)):
            self._wheels[_id].send_command(wheel_angle, wheel_distance, moving_time)
        time.sleep(moving_time/1000.0)

    def turn_in_place_and_move(self, angle, distance, turning_time=None, moving_time=None):
        if not turning_time:
            turning_time = abs(angle * 2000.0)
        if not moving_time:
            moving_time = abs(distance * 7000.0)
        self.turn_in_place(angle, turning_time)
        for _id in range(PlatformMath.WHEEL_NUM):
            self._wheels[_id].send_command(0.0, 0.0, 5)
        time.sleep(1)
        for _id in range(PlatformMath.WHEEL_NUM):
            self._wheels[_id].send_command(0.0, distance, moving_time)
        time.sleep(moving_time/1000.0)



def main():
    rospy.init_node('platform')
    wheels = []
    wheel_input_topics = []
    wheel_output_topics = []
    wheel_translations = []
    wheel_base_links = []
    wheel_links = []
    for _id in range(PlatformMath.WHEEL_NUM):
        wheel_input_topics.append(rospy.get_param(f"~wheel{_id+1}_output_topic"))
        wheel_output_topics.append(rospy.get_param(f"~wheel{_id+1}_input_topic"))

        raw_translation_string = rospy.get_param(f"~wheel{_id+1}_translation")
        xStr, yStr, zStr, RStr, PStr, YStr, base_link, wheel_link = raw_translation_string.split(' ')
        wheel_base_links.append(base_link)
        wheel_links.append(wheel_link)
        
        wheel_translations.append(Meta().xyzRPY2TransformStamped(float(xStr), float(yStr), float(zStr), float(RStr), float(PStr), float(YStr)))
        
    
    tf2_base_link = rospy.get_param("~tf2_base_link")
    platform = Platform(wheel_input_topics, wheel_output_topics, wheel_translations, wheel_base_links, wheel_links)
    rospy.spin()
    # signal.signal(signal.SIGINT, op.stop)
    # op.start()