#!/usr/bin/env python3
import time
import math
from py_omni_platform.controller import PlatformController

moves = [
    (0.1, 500, 0, 1000),
    (0.1, 500, math.radians( 45), 1000),
    (0.1, 500, math.radians(-45), 2000),
]

if __name__ == '__main__':
    controller = PlatformController()
    controller.start()
    while True:
        for move in moves:
            controller.turn_and_move(*move)
        print("All the moves ended, will wait for 10s")
        time.sleep(10)
    controller.join()