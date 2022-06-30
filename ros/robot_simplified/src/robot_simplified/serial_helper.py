import serial
try:
    import queue
except ImportError:
    import Queue as queue
import traceback
from threading import Thread
import time

import rospy

class SerialMock():
    def __init__(self, *args, **kwargs):
        self.__last_timestamp = 0
        rospy.loginfo("SerialMock.__init__:args:{} kwargs:{}".format(str(args), str(kwargs)))

    def __get_time_ms(self):
        return int(round(time.time() * 1000))

    def inWaiting(self):
        return True

    def readline(self):
        current_timestamp = self.__get_time_ms()
        if current_timestamp - self.__last_timestamp > 1:
            self.__last_timestamp = current_timestamp
            return "".encode('ascii')

    def write(self, *args, **kwargs):
        rospy.loginfo("SerialMock.write:args:{} kwargs:{}".format(str(args), str(kwargs)))
        
class SerialWrapper():

    def __init__(self, fpath, baudrate=115200):
        self._fpath = fpath
        self.__baudrate = baudrate
        try:
            self.serial = serial.Serial(fpath, baudrate, timeout=0.1)
        except Exception:
            tb = traceback.format_exc()
            rospy.loginfo(str(tb))
            self.serial = SerialMock(fpath, baudrate)

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