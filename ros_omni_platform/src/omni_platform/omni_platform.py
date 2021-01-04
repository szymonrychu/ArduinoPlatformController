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
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String

moves = [
    # (0.1, 500, math.radians( 45), 1000),
    # (0.1, 500, math.radians(-45), 2000),
    (0.1, 500, 0, 1000),
    (-0.1, 500, 0, 1000),
    (0, 500, math.radians( 45), 1000),
    (0, 500, math.radians(-45), 2000),
]

class OmniPlatform():

    def __init__(self):
        self._controller = PlatformController()

    def start(self):
        self._controller.start()
        while self._controller.running:
            controller.turn_and_move(*moves[move_num])
            time.sleep(5)
            move_num = (move_num+1)%len(moves)

    def stop(self):
        controller.join()

op = OmniPlatform()
signal.signal(signal.SIGINT, op.stop)
op.start()