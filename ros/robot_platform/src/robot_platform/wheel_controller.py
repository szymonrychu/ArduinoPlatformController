#!/usr/bin/env python3

import signal
import time
import math
import numpy as np
import json

from .odometry_helpers import PlatformStatics
import rospy
import tf_conversions
import tf2_ros

from math import pi as PI
from typing import Optional
from uuid import UUID

from robot_platform.msg import PlatformStatus, MoveRequest
from std_msgs.msg import String, Duration
from geometry_msgs.msg import Point, PoseStamped, Pose, TransformStamped, Point, PointStamped
from sensor_msgs.msg import BatteryState, NavSatFix, NavSatStatus, Imu

from .ros_helpers import ROSNode
from .log_utils import env2log
from .message_utils import parse_response, Request
from .serial_utils import SerialWrapper
from .tf_helpers import *


class SerialROSNode(ROSNode, SerialWrapper):

    def __init__(self):
        ROSNode.__init__(self)
        serial_dev = rospy.get_param('~serial_dev')
        serial_baudrate = rospy.get_param('~serial_baudrate')
        SerialWrapper.__init__(self, serial_dev, serial_baudrate)
    
    def start(self):
        rospy.Timer(rospy.Duration(0.001), self.__handle_serial)
        ROSNode.start(self)

    def __handle_serial(self, *_args, **_kwargs):
        raw_data = self.read_data()
        if raw_data:
            self.parse_serial(raw_data)

    def parse_serial(self, raw_data):
        pass

class WheelController(SerialROSNode):

    def __init__(self):
        SerialROSNode.__init__(self)
        self._message_counter = 0
        self._motor_distances = [0.0] * PlatformStatics.MOTOR_NUM
        self._header_frame_id = rospy.get_param('~header_frame_id')

        raw_input_topic = rospy.get_param('~raw_input_topic')
        raw_output_topic = rospy.get_param('~raw_output_topic')
        rospy.Subscriber(raw_input_topic, String, self._write_raw_data)
        self._raw_log_publisher = rospy.Publisher(raw_output_topic, String)

        wheel_positions_input_topic = rospy.get_param('~wheel_positions_input_topic')
        platform_status_output_topic = rospy.get_param('~platform_status_output_topic')
        battery_state_output_topic = rospy.get_param('~battery_state_output_topic')
        gps_state_output_topic = rospy.get_param('~gps_state_output_topic')
        imu_state_output_topic = rospy.get_param('~imu_state_output_topic')

        rospy.Subscriber(wheel_positions_input_topic, MoveRequest, self._handle_wheel_inputs)
        self._platform_status_publisher = rospy.Publisher(platform_status_output_topic, PlatformStatus)
        self._battery_state_publisher = rospy.Publisher(battery_state_output_topic, BatteryState)
        self._gps_state_publisher = rospy.Publisher(gps_state_output_topic, NavSatFix)
        self._imu_state_publisher = rospy.Publisher(imu_state_output_topic, Imu)

        self._tf2_broadcaster = tf2_ros.TransformBroadcaster()

        result = self.write_data('{"move_duration":1,"motor1":{"angle":0.0},"motor2":{"angle":0.0},"motor3":{"angle":0.0},"motor4":{"angle":0.0}}')
        if not result:
            self.stop()

    def _handle_wheel_inputs(self, raw_data:MoveRequest):
        r = Request.from_MoveRequest(raw_data)
        json_r = r.model_dump_json(exclude_none=True, exclude_unset=True)
        # rospy.loginfo(f"requesting: {str(r)}, '{json_r}'")
        if not self.write_data(json_r):
            self.stop()

    def parse_serial(self, raw_data:String):
        rospy_time_now = rospy.Time.now()

        response = parse_response(raw_data)
        if not response:
            return
        self._current_move_duration = response.move_duration

        raw_string = String()
        raw_string.data = raw_data
        self._raw_log_publisher.publish(raw_string)

        platform_status = response.parse_ROS(self._header_frame_id, rospy_time_now)

        self._platform_status_publisher.publish(platform_status)
        self._battery_state_publisher.publish(platform_status.battery)
        self._imu_state_publisher.publish(platform_status.imu)
        if platform_status.gps:
            self._gps_state_publisher.publish(platform_status.gps)

        if self._message_counter == 0:
            rospy.loginfo(f"Battery level: {response.battery.voltage}V")
        self._message_counter = (self._message_counter + 1) % 100

        transforms = [
            create_static_transform('base', 'scan', 0.0, 0.0, 0, 0, 0, math.pi, rospy_time_now)
        ]
        for c, (m_x, m_y), motor_status in zip([c for c in range(PlatformStatics.MOTOR_NUM)], PlatformStatics.ROBOT_MOTORS_DIMENSIONS, response.motor_list):
            transforms.append(create_static_transform('base', f"motor{c+1}base", m_x, m_y, 0, 0, 0, 0, rospy_time_now))
            transforms.append(create_static_transform(f"motor{c+1}base", f"motor{c+1}servo", 0, 0, 0, 0, 0, -motor_status.angle, rospy_time_now))
            self._motor_distances[c] += motor_status.distance
            motor_twist = motor_status.distance / PlatformStatics.WHEEL_RADIUS
            transforms.append(create_static_transform(f"motor{c+1}servo", f"motor{c+1}wheel", 0, 0, 0, 0, motor_twist, 0, rospy_time_now))
        
        for transform in transforms:
            self._tf2_broadcaster.sendTransform(transform)


    def _write_raw_data(self, ros_data:String):
        self.write_data(ros_data.data)

def main():
    platform = WheelController()
    signal.signal(signal.SIGINT, platform.stop)
    signal.signal(signal.SIGTERM, platform.stop)
    platform.start()

