import serial
try:
    import queue
except ImportError:
    import Queue as queue
import traceback
from threading import Thread, Lock
import time

import rospy

def time_ms():
    return round(time.time() * 1000)

import rospy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped


class TransformBroadcaster:
    """
    :class:`TransformBroadcaster` is a convenient way to send transformation updates on the ``"/tf"`` message topic.
    """

    def __init__(self, queue_size=100):
        self.pub_tf = rospy.Publisher("/tf", TFMessage, queue_size=queue_size)

    def sendTransform(self, transform):
        """
        Send a transform, or a list of transforms, to the Buffer associated with this TransformBroadcaster.

        :param transform: A transform or list of transforms to send.
        """
        if not isinstance(transform, list):
            transform = [transform]
        self.pub_tf.publish(TFMessage(transform))




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
        
class SerialWrapper():

    def __init__(self, fpath, baudrate=115200):
        self._fpath = fpath
        try:
            self.serial = serial.Serial(fpath, baudrate, timeout=0.1)
        except Exception:
            tb = traceback.format_exc()
            rospy.loginfo(str(tb))
        self._rate = Rate()

    @property
    def message_rate(self):
        return self._rate.mean_rate

    def data_available(self):
        return self.serial.inWaiting()

    def read_data(self):
        raw_data = None
        try:
            if self.data_available():
                raw_line = self.serial.readline()
                if raw_line is not None:
                    raw_data = raw_line.decode('ascii')
                    if raw_data != '':
                        if raw_data[-1] == '\n':
                            raw_data = raw_data[:-1]
                    self._rate.update()
        except serial.SerialTimeoutException:
            pass
        except UnicodeDecodeError:
            rospy.loginfo('cannot parse "{}"'.format(raw_data))
            self.repair_serial()
        return raw_data

    def write_data(self, raw_data):
        try:
            self.serial.write('{}\n'.format(raw_data).encode())
            return True
        except TypeError:
            tb = traceback.format_exc()
            rospy.loginfo(str(tb))
            self._repair_serial()

    def repair_serial(self):
        try:
            self.serial.close()
            self.serial = None
        except Exception:
            tb = traceback.format_exc()
            rospy.loginfo(str(tb))
        SerialWrapper.__init__(self, self._fpath, self.__baudrate)