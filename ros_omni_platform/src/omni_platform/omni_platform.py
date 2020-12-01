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

from controller import PlatformController

import rospy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String

moves = [
    (0.1, 500, 0, 1000),
    (0.1, 500, math.radians( 45), 1000),
    (0.1, 500, math.radians(-45), 2000),
]


def main():
    controller = PlatformController()
    controller.start()
    while True:
        for move in moves:
            controller.turn_and_move(*move)
        rospy.loginfo("All the moves ended, will wait for 10s")
        time.sleep(10)
    controller.join()