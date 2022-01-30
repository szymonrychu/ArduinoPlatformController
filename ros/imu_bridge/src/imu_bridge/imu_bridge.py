#!/usr/bin/env python3
from .serial_helper import SerialWrapper
import rospy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import NavSatFix, NavSatStatus, BatteryState
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
        self._battery_pub = rospy.Publisher('/battery', BatteryState, queue_size=10)
        self._gps_pub = rospy.Publisher('/gps', NavSatFix, queue_size=10)

    def _stabilize_quat(self, q1):
        roll, pitch, yaw = euler_from_quaternion([q1[1], q1[2], q1[3], q1[0]]) # wxyz -> xyzw
        q2 = quaternion_from_euler(0, 0, yaw) # xyzw
        return [q2[3], q2[0], q2[1], q2[2]] # xyzw -> wxyz

    def _parse(self, data):
        rospy_time_now = rospy.Time.now()
        _q, _raw_data = data.split(':')
        _spitted_raw_data = [s for s in _raw_data.split(',')]
        quat_WXYZ = [ 
            float(_spitted_raw_data[0]),
            float(_spitted_raw_data[1]),
            float(_spitted_raw_data[2]),
            float(_spitted_raw_data[3])
        ]
        voltage = float(_spitted_raw_data[4])

        nav_sat_status = NavSatStatus()
        nav_sat_status.service = NavSatStatus.SERVICE_GPS;

        nav_sat_fix = NavSatFix()
        nav_sat_fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN;
        nav_sat_fix.header.stamp = rospy_time_now
        nav_sat_fix.header.frame_id = self._tf2_base_link
        gps_fix_Q = int(_spitted_raw_data[5] or 0)
        gps_satellites = int(_spitted_raw_data[6] or 0)
        if gps_fix_Q > 0 and gps_satellites > 0:
            gps_latitude = float(_spitted_raw_data[7])
            gps_longitude = float(_spitted_raw_data[8])
            gps_speed = float(_spitted_raw_data[9])
            gps_angle = float(_spitted_raw_data[10])
            gps_altitude = float(_spitted_raw_data[11])
            nav_sat_status.status = NavSatStatus.STATUS_FIX;
            nav_sat_fix.status = nav_sat_status
            nav_sat_fix.latitude = gps_latitude
            nav_sat_fix.longitude = gps_longitude
            nav_sat_fix.altitude = gps_altitude
        else:
            nav_sat_status.status = NavSatStatus.STATUS_NO_FIX;

        battery_state = BatteryState()
        battery_state.header.stamp = rospy_time_now
        battery_state.header.frame_id = self._tf2_base_link
        if voltage > 0:
            battery_state.present = True
            battery_state.voltage = voltage
            battery_state.percentage = min(voltage / 13.00, 13.00)
            if voltage > 13:
                battery_state.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
            else:
                battery_state.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
        else:
            battery_state.present = False
            battery_state.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
        
        t1 = TransformStamped()
        t1.header.stamp = rospy_time_now
        t1.header.frame_id = self._tf2_base_link
        t1.child_frame_id = self._tf2_output
        t1.transform.translation.x = 0
        t1.transform.translation.y = 0
        t1.transform.translation.z = 0
        t1.transform.rotation.w = quat_WXYZ[0]
        t1.transform.rotation.x = quat_WXYZ[1]
        t1.transform.rotation.y = quat_WXYZ[2]
        t1.transform.rotation.z = quat_WXYZ[3]

        stabilized_q = self._stabilize_quat(quat_WXYZ)
        
        t2 = TransformStamped()
        t2.header.stamp = rospy_time_now
        t2.header.frame_id = self._tf2_base_link
        t2.child_frame_id = self._tf2_output_stabilized
        t2.transform.translation.x = 0
        t2.transform.translation.y = 0
        t2.transform.translation.z = 0
        t2.transform.rotation.w = stabilized_q[0]
        t2.transform.rotation.x = stabilized_q[1]
        t2.transform.rotation.y = stabilized_q[2]
        t2.transform.rotation.z = stabilized_q[3]

        self._gps_pub.publish(nav_sat_fix)
        self._battery_pub.publish(battery_state)
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