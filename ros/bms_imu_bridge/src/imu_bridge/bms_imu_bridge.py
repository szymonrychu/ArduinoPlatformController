#!/usr/bin/env python3
import serial
import signal
import time
from threading import Thread


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
        self.__running = True
        self._tf_broadcaster = tf2_ros.TransformBroadcaster()

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

    def __parse(self, raw_data):
        try:
            qw, qx, qy, qz = ( float(d) for d in raw_data.split(',') )
            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = '/base_link'
            t.child_frame_id = '/imu'
            t.transform.translation.x = 0
            t.transform.translation.y = 0
            t.transform.translation.z = 0
            t.transform.rotation.w = qw
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            self._tf_broadcaster.sendTransform(t)
            rospy.loginfo(f"Publishing IMU tf2 [{qx}, {qy}, {qz}, {qw}]")
        except ValueError:
            rospy.logwarn(f"Error Parsing: {raw_data}")


def main():
    rospy.init_node("bms_imu_bridge")

    imuObj = TF2ROSIMU()
    imuObj.handle_serial()
