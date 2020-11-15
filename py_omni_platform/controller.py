from .serial_helper import ThreadedSerialWrapper
from .platform_math import PlatformMath
from .platform_parser import PlatformParser

try:
    import queue
except ImportError:
    import Queue as queue
import time

class PlatformCommands():

    def move_command(self, angle, distance, time):
        return f"G11 {distance} {angle} {time}"

class PlatformController(PlatformMath, PlatformCommands):

    def __init__(self):
        receive_queue = queue.Queue()
        self.__threads = []
        for _id, wheel_serial_id in enumerate(PlatformController.WHEELS):
            self.__threads.append(ThreadedSerialWrapper(wheel_serial_id, _id, receive_queue))
        self.__threads.append(PlatformParser(receive_queue))

    def start(self):
        for th in self.__threads:
            th.start()

    def join(self, *args, **kwargs):
        for th in self.__threads:
            th.join(*args, **kwargs)

    def turn_and_move(self, distance, moving_time, angle, turning_time):
        distances = []
        angles = []
        for angle, wheel_distance in self.compute_angles_distances():
            angles.append(angle)
            distances.append(wheel_distance)
        for wheel_id, angle in enumerate(angles):
            self.__threads[wheel_id].write_data(self.move_command(angle, 0, turning_time))
        time.sleep(turning_time/1000.0)

        for wheel_id, wheel_distance in enumerate(distances):
            self.__threads[wheel_id].write_data(self.move_command(angle, wheel_distance, moving_time))
        time.sleep(moving_time/1000.0)

    def turn_in_place(self, distance, moving_time):
        raise NotImplementedError("XD")

