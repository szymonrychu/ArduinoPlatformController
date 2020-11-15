#!/usr/bin/env python3
import time
from py_omni_platform.controller import PlatformController

moves = [
    (1, 3000, 0, 0),
    # (1,  45, 1000, 3000),
    # (1, -45, 2000, 3000),
]

if __name__ == '__main__':
    controller = PlatformController()
    controller.turn_and_move((0, 0, 0, 1000))
    while True:
        for move in moves:
            controller.turn_and_move(*move)
        print("All the moves ended, will wait for 10s")
        time.sleep(10)