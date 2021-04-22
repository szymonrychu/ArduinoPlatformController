#!/usr/bin/env python3
import serial
try:
    import queue
except ImportError:
    import Queue as queue
import traceback
import time
import re
import math
import signal

from .controller import PlatformController

import rospy
import tf_conversions
from geometry_msgs.msg import Twist, Vector3, PoseStamped, Pose
from std_msgs.msg import String
import logging



class OmniPlatform(PlatformController):
    MOVES = [
        # (0.1, 500, math.radians( 45), 1000),
        # (0.1, 500, math.radians(-45), 2000),
        (0.3, 1500, 0, 1000),
        (-0.3, 1500, 0, 1000),
        (0.2, 1000, math.radians( 45), 1000),
        (-0.2, 1000, math.radians( -45), 1000),
        (0, 500, 0, 1000),
        (0, 500, 0, 2000),
        (-0.2, 1000, math.radians( 45), 1000),
        (0.2, 1000, math.radians( -45), 1000),
    ]

    def __init__(self):
        rospy.init_node('omni_platform')
        PlatformController.__init__(self)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.__callback)
        self._rate = rospy.Rate(10)
        self._move_num = 0
        self._move_queue = queue.Queue()
        self._current_pose = Pose()
        self._current_pose.position.x = 0
        self._current_pose.position.y = 0
        self._current_pose.position.z = 0

    def __callback(self, data):
        dx = data.pose.position.x - self._current_pose.position.x
        dy = data.pose.position.y - self._current_pose.position.y
        dz = data.pose.position.z - self._current_pose.position.z

        r, p, y = tf_conversions.transformations.euler_from_quaternion([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])

        angle = math.atan2(dy, dx)
        distance = math.sqrt(dx*dx + dy*dy)
        rospy.loginfo(f"a/d:{angle}/{distance}")

        self.turn_in_place_and_move(angle, distance) #, 2000, 3000)

    def start(self):
        PlatformController.start(self)
        while self.running:
            rospy.spin()
            # self.turn_and_move(*OmniPlatform.MOVES[self._move_num])
            # self._move_num = (self._move_num+1)%len(OmniPlatform.MOVES)
            self._rate.sleep()

    def stop(self, *args, **kwargs):
        PlatformController.join()


def main():
    op = OmniPlatform()
    signal.signal(signal.SIGINT, op.stop)
    op.start()