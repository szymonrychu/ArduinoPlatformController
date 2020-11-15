#!/usr/bin/env python3
import time
import math
from py_omni_platform.controller import PlatformController

moves = [
    (1, 3000, 0, 100),
    (1, 3000, math.radians( 45), 1000),
    (1, 3000, math.radians(-45), 2000),
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