from mimetypes import init
from threading import Lock
import time
import os

import rospy
from geometry_msgs.msg import Quaternion

ROBOT_WIDTH_M = 0.23

_env2log_name = 'ROS_LOG_LEVEL'
_env2log = {
    'DEBUG': rospy.DEBUG,
    'INFO':  rospy.INFO,
    'WARN':  rospy.WARN,
    'ERROR': rospy.ERROR,
    'FATAL': rospy.FATAL
}
def env2log():
    try:
        return _env2log[os.getenv(_env2log_name, 'INFO')]
    except Exception:
        return rospy.INFO

def time_ms():
    return round(time.time() * 1000)

class Rate():

    def __init__(self, max_rates = 100):
        self.__max_rates = max_rates
        self.__lock = Lock()
        self.__last_rates = []
        self.__last_timestamp = 0.0

    def update(self):
        current_time = time_ms()
        if self.__last_timestamp != 0:
            with self.__lock:
                time_delta = current_time - self.__last_timestamp
                self.__last_rates.append(time_delta)
                while len(self.__last_rates) > self.__max_rates:
                    self.__last_rates.pop()
        self.__last_timestamp = current_time

    @property
    def mean_rate(self, min_rates = 100):
        with self.__lock:
            rates_len = len(self.__last_rates)
            if rates_len >= min_rates:
                return sum(self.__last_rates) / float(rates_len)
        return -1.0

class SupressedLog():

    def __init__(self, suppression_time_s=0.0):
        self.__suppresion_time = suppression_time_s
        self.__last_timestamp = 0.0

    def handler(self, log_handler, message):
        current_time = time_ms()
        if current_time - self.__last_timestamp > self.__suppresion_time * 1000.0:
            self.__last_timestamp = current_time
            log_handler(message)


class RobotQuaternion(Quaternion):

    @staticmethod
    def from_arr(arr):
        return RobotQuaternion(arr[0], arr[1], arr[2], arr[3])

    @staticmethod
    def from_q(q):
        return RobotQuaternion(q.x, q.y, q.z, q.w)

    def to_arr(self):
        return [self.x, self.y, self.z, self.w]
    
    def q(self):
        return Quaternion(self.x, self.y, self.z, self.w)

class Latch():

    def __init__(self, initial_state = True):
        self.__lock = Lock()
        with self.__lock:
            self.__primed = False
            self.__initial_state = initial_state
            self.__state = initial_state
            self.__last_state = self.__initial_state

    def set_True(self):
        self.__primed = True
        self.__state = self.__initial_state
    
    def reset(self):
        with self.__lock:
            self.__primed = False
            self.__state = self.__initial_state
            self.__last_state = self.__initial_state

    def state(self):
        with self.__lock:
            return self.__primed and self.__initial_state == self.__state
    
    def update(self, new_state):
        '''
        Will return True only if:
            1. there was a change in new_state vs initial_state
            2. the new_state was set back to it's initial state
        '''
        with self.__lock:
            if self.__initial_state != new_state:
                self.__primed = True

            if self.__primed:
                self.__state = new_state
                
        return self.state()