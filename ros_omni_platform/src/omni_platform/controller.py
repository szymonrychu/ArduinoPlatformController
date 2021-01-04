from .serial_helper import ThreadedSerialWrapper
from .platform_math import PlatformMath
from .tf2_publisher import TF2PlatformPublisher

try:
    import queue
except ImportError:
    import Queue as queue
import time

import rospy

class PlatformCommands():

    def move_command(self, angle, distance, time):
        return "G11 {} {} {}".format(distance, angle, time)

class PlatformController(PlatformMath, PlatformCommands):

    def __init__(self):
        PlatformMath.__init__(self)
        rospy.init_node('robot_node')
        receive_queue = queue.Queue()
        self.__threads = []
        for _id, wheel_serial_id in enumerate(PlatformController.WHEELS):
            self.__threads.append(ThreadedSerialWrapper(wheel_serial_id, _id, receive_queue))
        self.__threads.append(TF2PlatformPublisher(receive_queue))

    @property
    def running(self):
        running = []
        for t in self.__threads:
            running.append(t.running)
        return all(running)

    def start(self):
        for th in self.__threads:
            th.start()

    def join(self, *args, **kwargs):
        for th in self.__threads:
            th.join(*args, **kwargs)

    def turn_and_move(self, distance, moving_time, angle, turning_time):
        distances = []
        angles = []
        for wheel_angle, wheel_distance in self.compute_angles_distances(angle, distance):
            angles.append(wheel_angle)
            distances.append(wheel_distance)
        
        if turning_time > 0:
            for wheel_id, wheel_angle in enumerate(angles):
                move_command = self.move_command(wheel_angle, 0, turning_time)
                print("{}: {}".format(str(wheel_id+1), move_command))
                self.__threads[wheel_id].write_data(move_command)
            time.sleep(turning_time/1000.0)

        for wheel_id, (wheel_angle, wheel_distance) in enumerate(zip(angles, distances)):
            move_command = self.move_command(wheel_angle, wheel_distance, moving_time)
            print("{}: {}".format(str(wheel_id+1), move_command))
            self.__threads[wheel_id].write_data(move_command)
        time.sleep(moving_time/1000.0)

    def turn_in_place(self, distance, moving_time):
        raise NotImplementedError("XD")

