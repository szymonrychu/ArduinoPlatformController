#!/usr/bin/env python3

import signal
import numpy as np

import rospy
import tf_conversions
import tf2_ros

from math import pi as PI
from typing import Optional
from uuid import UUID

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose, TransformStamped, Point
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

class RobotPlatformRawSerialROSNode(SerialROSNode):

    def __init__(self):
        SerialROSNode.__init__(self)
        self._current_pose = Pose()
        self._odom_handler = OdomHandler()
        self._currently_processed_move_uuid = None

        self._map_frame_id = rospy.get_param('~map_frame_id')
        self._base_link_id = rospy.get_param('~base_link_frame_id')
        self._base_link_stabilised_id = rospy.get_param('~base_link_stabilised_frame_id')
        self._scan_link_id = rospy.get_param('~scan_link_frame_id')

        raw_input_topic = rospy.get_param('~raw_input_topic')
        rospy.Subscriber(raw_input_topic, String, self._write_raw_data)
        raw_output_topic = rospy.get_param('~raw_output_topic')
        self._raw_log_publisher = rospy.Publisher(raw_output_topic, String)

        goal_input_topic = rospy.get_param('~goal_move_input_topic')
        rospy.Subscriber(goal_input_topic, PoseStamped, self.handle_goal_pose_input_data)

        battery_state_output_topic = rospy.get_param('~battery_state_output_topic')
        self._battery_state_publisher = rospy.Publisher(battery_state_output_topic, BatteryState)
        gps_state_output_topic = rospy.get_param('~gps_state_output_topic')
        self._gps_state_publisher = rospy.Publisher(gps_state_output_topic, NavSatFix)
        imu_state_output_topic = rospy.get_param('~imu_state_output_topic')
        self._imu_state_publisher = rospy.Publisher(imu_state_output_topic, Imu)
        odom_state_output_topic = rospy.get_param('~odom_state_output_topic')
        self._odom_state_publisher = rospy.Publisher(odom_state_output_topic, PoseStamped)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster()

    def parse_serial(self, raw_data:String):
        rospy_time_now = rospy.Time.now()

        response = parse_response(raw_data)
        if not response:
            return

        raw_string = String()
        raw_string.data = raw_data
        self._raw_log_publisher.publish(raw_string)

        rospy.logdebug(f'Raw platform output:\n{response.model_dump_json()}')
        self._battery_state_publisher.publish(response.battery.parse_ROS_Battery(self._base_link_id, rospy_time_now))
        self._imu_state_publisher.publish(response.imu.parse_ROS_IMU(self._base_link_id, rospy_time_now))
        if response.gps:
            self._gps_state_publisher.publish(response.gps.parse_ROS_GPS(self._base_link_id, rospy_time_now))

        # self._tf_broadcaster.sendTransform(create_static_transform(self._base_link_id, 'motor1_base', X=-0.2, Y=0.2))
        # self._tf_broadcaster.sendTransform(response.motor1.parse_ROS_TF('motor1_base', 'motor1'))
        # self._tf_broadcaster.sendTransform(create_static_transform(self._base_link_id, 'motor2_base', X=0.2, Y=0.2))
        # self._tf_broadcaster.sendTransform(response.motor2.parse_ROS_TF('motor2_base', 'motor2'))
        # self._tf_broadcaster.sendTransform(create_static_transform(self._base_link_id, 'motor3_base', X=-0.2, Y=-0.2))
        # self._tf_broadcaster.sendTransform(response.motor3.parse_ROS_TF('motor3_base', 'motor3'))
        # self._tf_broadcaster.sendTransform(create_static_transform(self._base_link_id, 'motor4_base', X=-0.2, Y=-0.2))
        # self._tf_broadcaster.sendTransform(response.motor4.parse_ROS_TF('motor4_base', 'motor4'))

        # self._tf_broadcaster.sendTransform(create_static_transform(self._base_link_id, 'pan_base', Z=0.1))
        # self._tf_broadcaster.sendTransform(response.pan.parse_ROS_TF('pan_base', 'pan'))
        # self._tf_broadcaster.sendTransform(response.tilt.parse_ROS_TF('pan', 'tilt'))

        pose = self._odom_handler.handle_motor_updates(response)
        p = Point()
        p.z = 0.1

        self._odom_state_publisher.publish(pose_to_pose_stamped(pose, self._base_link_id, p, rospy_time_now))
        transform = pose_to_transform_stamped(pose, self._map_frame_id, self._base_link_id, p, rospy_time_now)
        transform_stabilised = pose_to_transform_stabilised_stamped(pose, self._map_frame_id, self._base_link_stabilised_id, p, rospy_time_now)

        self._tf_broadcaster.sendTransform(transform)
        self._tf_broadcaster.sendTransform(transform_stabilised)
        self._tf_broadcaster.sendTransform(create_static_transform(self._base_link_id, self._scan_link_id, X=0.1, Yaw=PI, timestamp=rospy_time_now))

    def _write_raw_data(self, ros_data):
        self.write_data(ros_data.data)

    def handle_goal_pose_input_data(self, goal_pose:PoseStamped):
        self.write_data(self._odom_handler.request_move(goal_pose))

def main():
    platform = RobotPlatformRawSerialROSNode()
    signal.signal(signal.SIGINT, platform.stop)
    signal.signal(signal.SIGTERM, platform.stop)
    platform.start()


