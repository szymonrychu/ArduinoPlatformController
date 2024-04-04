#!/usr/bin/env python3

import signal
import time
import math
import numpy as np

import rospy
import tf_conversions
import tf2_ros

from math import pi as PI
from typing import Optional
from uuid import UUID

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose, TransformStamped, Point, PointStamped
from sensor_msgs.msg import BatteryState, NavSatFix, NavSatStatus, Imu

from .log_utils import env2log
from .message_utils import parse_response, GPSStatus, StatusResponse, BatteryStatus, IMUStatus, MotorStatus, ServoStatus
from .odom_handler import OdomHandler
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


class OdometryHandlerROSNode(ROSNode):

    def __init__(self):
        ROSNode.__init__(self)
        self._current_pose = Pose()
        self._current_move_duration = 0
        self._message_counter = 0

        # self._map_frame_id = rospy.get_param('~map_frame_id')
        # self._base_link_id = rospy.get_param('~base_link_frame_id')
        # self._base_link_stabilised_id = rospy.get_param('~base_link_stabilised_frame_id')
        # self._scan_link_id = rospy.get_param('~scan_link_frame_id')
        

        gps_state_input_topic = rospy.get_param('~gps_state_input_topic')
        self._gps_state_publisher = rospy.Publisher(gps_state_input_topic, NavSatFix)
        imu_state_input_topic = rospy.get_param('~imu_state_input_topic')
        self._imu_state_publisher = rospy.Publisher(imu_state_input_topic, Imu)
        wheel_positions_input_topic = rospy.get_param('~wheel_positions_input_topic')
        rospy.Subscriber(wheel_positions_input_topic, WheelResponse, self._write_raw_data)

        odom_state_output_topic = rospy.get_param('~odom_state_output_topic')
        self._odom_state_publisher = rospy.Publisher(odom_state_output_topic, PoseStamped)

        self._tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.write_data('{"move_duration":1,"motor1":{"angle":0.0},"motor2":{"angle":0.0},"motor3":{"angle":0.0},"motor4":{"angle":0.0}}')


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
    platform = OdometryHandlerROSNode()
    signal.signal(signal.SIGINT, platform.stop)
    signal.signal(signal.SIGTERM, platform.stop)
    platform.start()


