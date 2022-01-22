#!/usr/bin/env python3
from .serial_helper import SerialWrapper
import rospy
from geometry_msgs.msg import TransformStamped
import tf
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import os
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


class Monitor(SerialWrapper):

    def __init__(self, serial_dev, baudrate, battery_topic, tf2_base_link, tf2_output, tf2_output_stabilized):
        SerialWrapper.__init__(self, serial_dev, baudrate=baudrate)
        self.__last_distance = 0.0
        self.__distance_set = False
        self._tf_broadcaster = tf2_ros.TransformBroadcaster()
        self._tf2_base_link = tf2_base_link
        self._tf2_output = tf2_output
        self._tf2_output_stabilized = tf2_output_stabilized
        self._battery_topic = battery_topic

    def _stabilize_quat(self, q1):
        roll, pitch, yaw = euler_from_quaternion([q1[1], q1[2], q1[3], q1[0]])
        q2 = quaternion_from_euler(0, 0, yaw)
        return [q2[1], q2[2], q2[3], q2[0]]

    def _parse(self, data):
        _q, quat_t = data.split(':')
        data = [ float(d) for d in quat_t.split(',') ]
        # qw, qx, qy, qz
        # battFull, systemON, powerON, chargingON
        # batteryVoltage, chargingVoltage
        # gpsFix, GPSfixQuality, GPSSatellites,
        # latitude, longitude, speed, angle, altitude
        
        t1 = TransformStamped()
        t1.header.stamp = rospy.Time.now()
        t1.header.frame_id = self._tf2_base_link
        t1.child_frame_id = self._tf2_output
        t1.transform.translation.x = 0
        t1.transform.translation.y = 0
        t1.transform.translation.z = 0
        t1.transform.rotation.w = data[0]
        t1.transform.rotation.x = data[1]
        t1.transform.rotation.y = data[2]
        t1.transform.rotation.z = data[3]

        stabilized_q = self._stabilize_quat(data)
        
        t2 = TransformStamped()
        t2.header.stamp = rospy.Time.now()
        t2.header.frame_id = self._tf2_base_link
        t2.child_frame_id = self._tf2_output
        t2.transform.translation.x = 0
        t2.transform.translation.y = 0
        t2.transform.translation.z = 0
        t2.transform.rotation.w = stabilized_q[0]
        t2.transform.rotation.x = stabilized_q[1]
        t2.transform.rotation.y = stabilized_q[2]
        t2.transform.rotation.z = stabilized_q[3]

        self._tf_broadcaster.sendTransform(t1)
        self._tf_broadcaster.sendTransform(t2)

    def process(self):
        while True:
            raw_data = self.read_data()
            if raw_data is not None:
                rospy.logdebug(f"received raw: {raw_data}")
                self._parse(raw_data)

def main():
    rospy.init_node('imu_bridge', log_level=env2log())

    serial_dev = rospy.get_param("~serial_dev")
    baudrate = rospy.get_param("~baudrate")
    battery_topic = rospy.get_param("~battery_topic")
    tf2_base_link = rospy.get_param("~tf2_base_link")
    tf2_output = rospy.get_param("~tf2_output")
    tf2_output_stabilized = rospy.get_param("~tf2_output_stabilized")


    Monitor(serial_dev, baudrate, battery_topic, tf2_base_link, tf2_output, tf2_output_stabilized).process()