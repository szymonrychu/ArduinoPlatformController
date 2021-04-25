#!/usr/bin/env python3
from .serial_helper import SerialWrapper
import rospy
from geometry_msgs.msg import TransformStamped
import tf
import tf2_ros


class Monitor(SerialWrapper):

    def __init__(self, serial_dev, baudrate, battery_topic, tf2_base_link, tf2_output):
        SerialWrapper.__init__(self, serial_dev, baudrate=baudrate)
        self.__last_distance = 0.0
        self.__distance_set = False
        self._tf_broadcaster = tf2_ros.TransformBroadcaster()
        self._tf2_base_link = tf2_base_link
        self._tf2_output = tf2_output
        self._battery_topic = battery_topic

    def _parse(self, data):
        data = ( float(d) for d in raw_data.split(',') )
        # qw, qx, qy, qz
        # battFull, systemON, powerON, chargingON
        # batteryVoltage, chargingVoltage
        # gpsFix, GPSfixQuality, GPSSatellites,
        # latitude, longitude, speed, angle, altitude
        
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self._tf2_base_link
        t.child_frame_id = self._tf2_output
        t.transform.translation.x = 0
        t.transform.translation.y = 0
        t.transform.translation.z = 0
        t.transform.rotation.w = data[0]
        t.transform.rotation.x = data[1]
        t.transform.rotation.y = data[2]
        t.transform.rotation.z = data[3]
        self._tf_broadcaster.sendTransform(t)

    def process(self):
        while True:
            raw_data = self.read_data()
            if raw_data is not None:
                rospy.logdebug(f"received raw: {raw_data}")
                self._parse(raw_data)

def main():
    rospy.init_node('bms_imu_bridge')

    serial_dev = rospy.get_param("~serial_dev")
    baudrate = rospy.get_param("~baudrate")
    battery_topic = rospy.get_param("~battery_topic")
    tf2_base_link = rospy.get_param("~tf2_base_link")
    tf2_output = rospy.get_param("~tf2_output")


    Monitor(serial_dev, baudrate, battery_topic, tf2_base_link, tf2_output).process()