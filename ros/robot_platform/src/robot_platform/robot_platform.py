#!/usr/bin/env python3
from .platform_math import PlatformMath

import signal
import rospy
import tf_conversions
import tf2_ros
from geometry_msgs.msg import Twist, Vector3, PoseStamped, TransformStamped, Pose2D
from std_msgs.msg import String
import logging
from copy import copy
from threading import Lock
import math
import time

import os
_env2log_name = 'ROS_LOG_LEVEL'
_env2log = {
    'DEBUG': rospy.DEBUG,
    'INFO':  rospy.INFO,
    'WARN':  rospy.WARN,
    'ERROR': rospy.ERROR,
    'FATAL': rospy.FATAL
}
def env2log():
    try:
        return _env2log[os.getenv(_env2log_name, 'INFO')]
    except Exception:
        return rospy.INFO



class Meta():

    def __init__(self, base_link, link):
        rospy.loginfo(f"{base_link} {link}")
        self._tf2_base_link = base_link
        self._tf2_link = link

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
        Meta.__init__(self, base_link, link)
        self._handler = data_handler
        self._id = _id
        self._static_translation = translation
        self.publisher = rospy.Publisher(output_topic, Vector3, queue_size=10)
        rospy.Subscriber(input_topic, Pose2D, self.__callback)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster()
    
    def __callback(self, distance_angle):
        self._handler(self._id+1, distance_angle)

    def send_transform(self):
        self._static_translation.header.stamp = rospy.Time.now()
        self._tf_broadcaster.sendTransform(self._static_translation)

    def send_command(self, distance, angle, time):
        v = Vector3()
        v.x = distance
        v.y = angle
        v.z = time
        self.publisher.publish(v)

class Platform(PlatformMath, Meta):

    def __init__(self, wheel_input_topics, wheel_output_topics, wheel_translations, wheel_base_links, wheel_links):
        Meta.__init__(self, "/base_link", "/map")
        self._wheels = []
        self._platform_transform = TransformStamped()
        self._platform_transform.header.frame_id = "/map"
        self._platform_transform.child_frame_id = "/base_link"
        self._distance_angles = [None] * Platform.WHEEL_NUM
        self._wheel_xy = [Pose2D() for _ in range(Platform.WHEEL_NUM)]
        for c in range(Platform.WHEEL_NUM):
            tr_x, tr_y, tr_z = self.WHEELS_TRANSLATIONS_XYZ[c]
            self._wheel_xy[c].x = tr_x
            self._wheel_xy[c].y = tr_y
        self._lock = Lock()
        for _id in range(Platform.WHEEL_NUM):
            self._wheels.append(Wheel(_id, wheel_input_topics[_id], wheel_output_topics[_id], wheel_translations[_id], wheel_base_links[_id], wheel_links[_id], self.wheel_output_hander))
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self._goal_callback)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster()
        self._previous_center_distance = Pose2D()

    def wheel_output_hander(self, _id, distance_angle):
        with self._lock:
            self._distance_angles[_id-1] = distance_angle
            if all(self._distance_angles):
                for c in range(Platform.WHEEL_NUM):
                    if c == 0 or c == 1:
                        self._wheel_xy[c].x += self._distance_angles[c].x * math.cos(self._distance_angles[c].y)
                        self._wheel_xy[c].y += self._distance_angles[c].x * math.sin(self._distance_angles[c].y)
                    else:
                        self._wheel_xy[c].x += -(self._distance_angles[c].x * math.cos(self._distance_angles[c].y))
                        self._wheel_xy[c].y += -(self._distance_angles[c].x * math.sin(self._distance_angles[c].y))

                raw_delta_distance = sum([self._distance_angles[c].x if c==0 or c==1 else -self._distance_angles[c].x for c in range(Platform.WHEEL_NUM)])/Platform.WHEEL_NUM
                # raw_delta_distance = sum([self._distance_angles[c].x for c in range(Platform.WHEEL_NUM)])

                debug_str = []
                for c in range(Platform.WHEEL_NUM):
                    debug_str.append(f"{100*self._wheel_xy[c].x:.4f}/{100*self._wheel_xy[c].y:.4f}")
                    # debug_str.append(f"{'U' if self._wheel_xy[c].x > 0 else 'D'}/{'R' if self._wheel_xy[c].y > 0 else 'L'}")
                # rospy.loginfo(' '.join(debug_str))
                
                front_back_vector = Pose2D()
                front_back_vector.x = (self._wheel_xy[0].x + self._wheel_xy[1].x) - (self._wheel_xy[2].x + self._wheel_xy[3].x)
                front_back_vector.y = (self._wheel_xy[0].y + self._wheel_xy[1].y) - (self._wheel_xy[2].y + self._wheel_xy[3].y)

                Y = 2*math.atan2(-front_back_vector.y, front_back_vector.x)

                self._platform_transform.transform.translation.x += raw_delta_distance * math.cos(Y)
                self._platform_transform.transform.translation.y += raw_delta_distance * math.sin(Y)

                q = tf_conversions.transformations.quaternion_from_euler(0.0, 0.0, Y)
                self._platform_transform.transform.rotation.x = q[0]
                self._platform_transform.transform.rotation.y = q[1]
                self._platform_transform.transform.rotation.z = q[2]
                self._platform_transform.transform.rotation.w = q[3]

                # rospy.loginfo(f"center: {self._platform_transform.transform.translation.x}, {self._platform_transform.transform.translation.y}, {Y}")
                self._platform_transform.header.stamp = rospy.Time.now()
                self._tf_broadcaster.sendTransform(self._platform_transform)
                for wheel in self._wheels:
                    wheel.send_transform()
                self._distance_angles = [None] * Platform.WHEEL_NUM
                
                

    def _goal_callback(self, data):
        rospy.loginfo(f"{data.pose.position.x} {data.pose.position.y} {data.pose.position.z}")
        dx = data.pose.position.x - self._platform_transform.transform.translation.x
        dy = data.pose.position.y - self._platform_transform.transform.translation.y
        dz = data.pose.position.z - self._platform_transform.transform.translation.z

        r, p, y = tf_conversions.transformations.euler_from_quaternion([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])

        angle = math.atan2(dy, dx)
        distance = math.sqrt(dx*dx + dy*dy)

        rospy.loginfo(f"Rotating {angle} and moving {distance}")

        self.turn_in_place_and_move(angle, distance)

    # def turn_and_move(self, distance, moving_time, angle, turning_time):
    #     distances = []
    #     angles = []
    #     for wheel_angle, wheel_distance in self.compute_angles_distances(angle, distance):
    #         angles.append(wheel_angle)
    #         distances.append(wheel_distance)
        
    #     if turning_time > 0:
    #         for _id, wheel_angle in enumerate(angles):
    #             self._wheels[_id].send_command(0, wheel_angle, turning_time)
    #         time.sleep(turning_time/1000.0)

    #     for _id, (wheel_angle, wheel_distance) in enumerate(zip(angles, distances)):
    #         self._wheels[_id].send_command(wheel_distance, wheel_angle, moving_time)
    #     time.sleep(moving_time/1000.0)

    def turn_in_place(self, angle, turning_time):
        angles = list(self.get_in_place_angles())
        distances = list(self.get_in_place_angle2distance(angle))

        rospy.loginfo("Turning wheels to [" + ",".join([str(a) for a in angles]) + "]")
        for _id, wheel_angle in enumerate(angles):
            self._wheels[_id].send_command(0.0, wheel_angle, 0.5)
        time.sleep(0.5)

        rospy.loginfo("Moving wheels with [" + ",".join([str(d) for d in distances]) + "]")
        for _id, (wheel_angle, wheel_distance) in enumerate(zip(angles, distances)):
            self._wheels[_id].send_command(wheel_distance, wheel_angle, turning_time)
        time.sleep(turning_time)

        rospy.loginfo("Turning wheels to [0,0,0,0]")
        for _id in range(PlatformMath.WHEEL_NUM):
            self._wheels[_id].send_command(0.0, 0.0, 0.5)
        time.sleep(0.5)

    def turn_in_place_and_move(self, angle, distance, turning_time=None, moving_time=None):
        if not turning_time:
            turning_time = abs(angle * 2.0)
        if not moving_time:
            moving_time = abs(distance * 2.0)
        
        self.turn_in_place(angle, turning_time)
        for _id in range(PlatformMath.WHEEL_NUM):
            self._wheels[_id].send_command(distance, 0.0, moving_time)
        time.sleep(moving_time)



def main():
    rospy.init_node('robot_platform', log_level=env2log())
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
        
        wheel_translations.append(Meta('/base_link', f"/wheel{_id+1}_pivot").xyzRPY2TransformStamped(float(xStr), float(yStr), float(zStr), float(RStr), float(PStr), float(YStr)))
        
    
    tf2_base_link = rospy.get_param("~tf2_base_link")
    platform = Platform(wheel_input_topics, wheel_output_topics, wheel_translations, wheel_base_links, wheel_links)
    rospy.spin()
    # signal.signal(signal.SIGINT, op.stop)
    # op.start()