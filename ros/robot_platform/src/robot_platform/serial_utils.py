import serial
import traceback
import time
from threading import Lock

import rospy
        
class SerialWrapper():

    def __init__(self, fpath, baudrate=115200):
        self._fpath = fpath
        self.__baudrate = baudrate
        self._serial = serial.Serial(fpath, baudrate)

    def read_data(self):
        raw_data = None
        try:
            if self._serial.inWaiting():
                raw_line = self._serial.readline()
                if raw_line is not None:
                    raw_data = raw_line.decode('ascii')
                    if raw_data != '':
                        if raw_data[-1] == '\n':
                            raw_data = raw_data[:-1]
        except UnicodeDecodeError:
            rospy.loginfo('cannot parse "{}"'.format(raw_data))
        return raw_data

    def write_data(self, raw_data):
        try:
            self._serial.write('{}\n'.format(raw_data).encode())
            return True
        except TypeError:
            return False
        except serial.SerialException:
            return False

class SafeSerialWrapper(SerialWrapper):

    def __init__(self, *args, **kwargs):
        self._read_lock = Lock()
        self._write_lock = Lock()
        super(SerialWrapper, self).__init__(*args, **kwargs)
    
    def read_data(self):
        with self._read_lock:
            return super(SerialWrapper, self).read_data()
    
    def write_data(self, raw_data):
        with self._write_lock:
            return super().write_data(raw_data)
