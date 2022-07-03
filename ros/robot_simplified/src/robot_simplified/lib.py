from threading import Lock
import time

import rospy

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
