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



class OmniPlatform():
    MOVES = [
        # (0.1, 500, math.radians( 45), 1000),
        # (0.1, 500, math.radians(-45), 2000),
        # (0.3, 1500, 0, 1000),
        # (-0.3, 1500, 0, 1000),
        # (0.2, 1000, math.radians( 45), 1000),
        # (-0.2, 1000, math.radians( -45), 1000),
        # (0, 500, 0, 1000),
        # (0, 500, 0, 2000),
        # (-0.2, 1000, math.radians( 45), 1000),
        # (0.2, 1000, math.radians( -45), 1000),
    ]

    def __init__(self):
        self._controller = PlatformController()
        self._move_num = 0

    def start(self):
        self._controller.start()
        while self._controller.running:
            self._controller.turn_and_move(*OmniPlatform.MOVES[self._move_num])
            self._move_num = (self._move_num+1)%len(OmniPlatform.MOVES)
            time.sleep(1)
        self.stop()

    def stop(self, *args, **kwargs):
        self._controller.join()


def main():
    op = OmniPlatform()
    signal.signal(signal.SIGINT, op.stop)
    op.start()