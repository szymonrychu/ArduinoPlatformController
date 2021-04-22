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
        self.__fpath = fpath
        self.__baudrate = baudrate
        try:
            self.serial = serial.Serial(fpath, baudrate)
        except Exception:
            tb = traceback.format_exc()
            rospy.loginfo(str(tb))
            self.serial = SerialMock(fpath, baudrate)

    def _read_data(self):
        raw_data = None
        try:
            if self.serial.inWaiting():
                raw_line = self.serial.readline()
                if raw_line is not None:
                    raw_data = raw_line.decode('ascii')
                    if raw_data != '':
                        if raw_data[-1] == '\n':
                            raw_data = raw_data[:-1]
        except UnicodeDecodeError:
            rospy.loginfo('cannot parse "{}"'.format(raw_data))
            self._repair_serial()
        return raw_data

    def write_data(self, raw_data):
        try:
            self.serial.write('{}\n'.format(raw_data).encode())
            return True
        except TypeError:
            tb = traceback.format_exc()
            rospy.loginfo(str(tb))
            self._repair_serial()

    def _repair_serial(self):
        try:
            self.serial.close()
            self.serial = None
        except Exception:
            tb = traceback.format_exc()
            rospy.loginfo(str(tb))
        SerialWrapper.__init__(self, self.__fpath, self.__baudrate)


class ThreadedSerialWrapper(Thread, SerialWrapper):

    def __init__(self, fpath, this_id, outgoing_queue, baudrate=115200):
        Thread.__init__(self, target=self.__handle_incoming_data)
        SerialWrapper.__init__(self, fpath, baudrate)
        self.__id = this_id
        self.__running = False
        self.__outgoing_queue = outgoing_queue

    def start(self):
        self.__running = True
        rospy.loginfo(f"ThreadedSerialWrapper starting!")
        Thread.start(self)
    
    @property
    def running(self):
        return self.__running

    def join(self, *args, **kwargs):
        self.__running = False
        Thread.join(self, *args, **kwargs)

    def __handle_incoming_data(self):
        while self.__running:
            raw_data = self._read_data()
            rospy.loginfo(f"received raw: {raw_data}")
            if raw_data is not None:
                self.__outgoing_queue.put((self.__id, raw_data))

class ThreadedSerialOutputHandler(Thread):

    def __init__(self, data_queue):
        Thread.__init__(self, target=self.__handle_incoming_data)
        self.__data_queue = data_queue
        self.__running = False

    def start(self):
        self.__running = True
        rospy.loginfo(f"ThreadedSerialOutputHandler starting!")
        Thread.start(self)
    
    @property
    def running(self):
        return self.__running

    def join(self, *args, **kwargs):
        self.__running = False
        Thread.join(self, *args, **kwargs)

    def __handle_incoming_data(self):
        while self.__running:
            _id, data = self.__data_queue.get()
            self.parse_serial(_id, data)

    def parse_serial(self, wheel_id, raw_data):
        pass