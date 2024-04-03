#!/usr/bin/env python3

import signal
import time
import math
import numpy as np
import json

import rospy
import tf_conversions
import tf2_ros

from math import pi as PI
from typing import Optional
from uuid import UUID

from robot_platform.msg import WheelRequest
from std_msgs.msg import String, Duration
from geometry_msgs.msg import Point, PoseStamped, Pose, TransformStamped, Point, PointStamped
from sensor_msgs.msg import BatteryState, NavSatFix, NavSatStatus, Imu

from .log_utils import env2log
from .message_utils import parse_response, GPSStatus, StatusResponse, BatteryStatus, IMUStatus, MotorStatus, ServoStatus
from .odom_handler import OdomHandler
from .serial_utils import SerialWrapper
from .tf_helpers import *

class ROSNode():

    def __init__(self, node_name='robot_platform'):
        rospy.init_node(node_name, log_level=env2log())

    def start(self):
        rospy.spin()

    def is_running(self):
        return not rospy.is_shutdown()

    def stop(self, reason='', *_args, **_kwargs):
        rospy.signal_shutdown(reason)


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

        raw_input_topic = rospy.get_param('~raw_input_topic')
        raw_output_topic = rospy.get_param('~raw_output_topic')
        rospy.Subscriber(raw_input_topic, String, self._write_raw_data)
        self._raw_log_publisher = rospy.Publisher(raw_output_topic, String)

        wheel_positions_input_topic = rospy.get_param('~wheel_positions_input_topic')
        battery_state_output_topic = rospy.get_param('~battery_state_output_topic')
        gps_state_output_topic = rospy.get_param('~gps_state_output_topic')
        imu_state_output_topic = rospy.get_param('~imu_state_output_topic')

        rospy.Subscriber(wheel_positions_input_topic, WheelRequest, self._write_raw_data)
        self._battery_state_publisher = rospy.Publisher(battery_state_output_topic, BatteryState)
        self._gps_state_publisher = rospy.Publisher(gps_state_output_topic, NavSatFix)
        self._imu_state_publisher = rospy.Publisher(imu_state_output_topic, Imu)

        self.write_data('{"move_duration":1,"motor1":{"angle":0.0},"motor2":{"angle":0.0},"motor3":{"angle":0.0},"motor4":{"angle":0.0}}')

    def _handle_raw_wheel_inputs(self, raw_data:WheelRequest):
        raw_request = {
            'move_duration': raw_data.duration,
            'motor1': {}
            'motor2': {}
            'motor3': {}
            'motor4': {}
        }

        if raw_data.motor1.angle_provided:
            raw_request['motor1']['angle'] = raw_data.motor1.angle
        if raw_data.motor2.angle_provided:
            raw_request['motor2']['angle'] = raw_data.motor2.angle
        if raw_data.motor3.angle_provided:
            raw_request['motor3']['angle'] = raw_data.motor3.angle
        if raw_data.motor4.angle_provided:
            raw_request['motor4']['angle'] = raw_data.motor4.angle

        if raw_data.motor1.distance_provided:
            raw_request['motor1']['angle'] = raw_data.motor1.distance
        if raw_data.motor2.distance_provided:
            raw_request['motor2']['angle'] = raw_data.motor2.distance
        if raw_data.motor3.distance_provided:
            raw_request['motor3']['angle'] = raw_data.motor3.distance
        if raw_data.motor4.distance_provided:
            raw_request['motor4']['angle'] = raw_data.motor4.distance
        self.write_data(json.dumps(raw_request))

    def parse_serial(self, raw_data:String):
        rospy_time_now = rospy.Time.now()

        response = parse_response(raw_data)
        if not response:
            return
        self._current_move_duration = response.move_duration

        raw_string = String()
        raw_string.data = raw_data
        self._raw_log_publisher.publish(raw_string)

        self._battery_state_publisher.publish(response.battery.parse_ROS_Battery(self._base_link_id, rospy_time_now))
        self._imu_state_publisher.publish(response.imu.parse_ROS_IMU(self._base_link_id, rospy_time_now))
        if response.gps:
            self._gps_state_publisher.publish(response.gps.parse_ROS_GPS(self._base_link_id, rospy_time_now))

        if self._message_counter == 0:
            rospy.loginfo(f"Battery level: {response.battery.voltage}V")
        self._message_counter = (self._message_counter + 1) % 100

    def _write_raw_data(self, ros_data:String):
        self.write_data(ros_data.data)

    def handle_goal_pose_input_data(self, goal_pose:PoseStamped):
        pose_delta = substract_points(goal_pose.pose.position, self._current_pose.position)

        roll, pitch, yaw = get_rpy_from_quaternion(goal_pose.pose.orientation)

        turn_angle = math.atan2(pose_delta.y, pose_delta.x)
        if abs(turn_angle) > 0.001:
            for move in self._odom_handler.turn_around_XY(turn_angle):
                raw_move = move.model_dump_json(exclude_none=True)
                rospy.loginfo(raw_move)
                self.write_data(raw_move)
                time.sleep(move.move_duration)
        else:
            rospy.loginfo(f"Ommiting turn in move request- reason: abs(turn_angle) = {turn_angle} <= 0.001")
        
        move_distance = math.sqrt(pose_delta.x*pose_delta.x + pose_delta.y*pose_delta.y)
        if abs(move_distance) > 0.01:
            for move in self._odom_handler.move_forward(move_distance):
                raw_move = move.model_dump_json(exclude_none=True)
                rospy.loginfo(raw_move)
                self.write_data(raw_move)
                time.sleep(move.move_duration)
        else:
            rospy.loginfo(f"Ommiting move_forward in move request- reason: abs(move_distance) = {move_distance} <= 0.001")

def main():
    platform = RobotPlatformRawSerialROSNode()
    signal.signal(signal.SIGINT, platform.stop)
    signal.signal(signal.SIGTERM, platform.stop)
    platform.start()


