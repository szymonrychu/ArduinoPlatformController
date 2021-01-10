#!/usr/bin/env python3
import serial
import signal
import time
from threading import Thread, Lock


import rospy
import tf
import tf2_ros
import geometry_msgs.msg
import tf_conversions

from .serial_helper import SerialWrapper

class TF2ROSIMU(SerialWrapper, Thread):

    def __init__(self):
        Thread.__init__(self, target=self.handle_serial)
        SerialWrapper.__init__(self, '/dev/serial/by-id/usb-Teensyduino_USB_Serial_7121500-if00', 115200)
        self.__running = False
        self.qx, self.qy, self.qz, self.qw = 0, 0, 0, 0
        self.__lock = Lock()

    def start(self):
        self.__running = True
        rospy.loginfo(f"TF2ROSIMU starting!")
        Thread.start(self)
    
    @property
    def running(self):
        return self.__running

    def join(self, *args, **kwargs):
        self.__running = False
        Thread.join(self, *args, **kwargs)

    def handle_serial(self):
        while self.__running:
            raw_data = self._read_data()
            if raw_data is not None:
                self.__parse(raw_data)

    @property
    def q(self):
        with self.__lock:
            return (self.qx, self.qy, self.qz, self.qw)

    def __parse(self, raw_data):
        try:
            with self.__lock:
                self.qw, self.qx, self.qy, self.qz = ( float(d) for d in raw_data.split(',') )
        except ValueError:
            rospy.logwarn(f"Error Parsing: {raw_data}")
