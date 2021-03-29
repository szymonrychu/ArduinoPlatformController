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



class OmniPlatform():
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
        rospy.init_node('robot_node')
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.__callback)
        self._controller = PlatformController()
        self._move_num = 0
        self._move_queue = queue.Queue()

        self._current_pose = Pose()
        self._current_pose.position.x = 0
        self._current_pose.position.y = 0
        self._current_pose.position.z = 0

    def __callback(self, data):
        delta_x = data.pose.position.x - self._current_pose.position.x
        delta_y = data.pose.position.y - self._current_pose.position.y
        delta_z = data.pose.position.z - self._current_pose.position.z

        rpy_angles = tf_conversions.transformations.euler_from_quaternion(data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w)
        print(rpy_angles)
        

    def start(self):
        self._controller.start()
        rospy.spin()

        while self._controller.running:
            # self._controller.turn_and_move(*OmniPlatform.MOVES[self._move_num])
            # self._move_num = (self._move_num+1)%len(OmniPlatform.MOVES)
            time.sleep(1)


    def stop(self):
        self._controller.join()


def main():
    op = OmniPlatform()
    signal.signal(signal.SIGINT, op.stop)
    op.start()