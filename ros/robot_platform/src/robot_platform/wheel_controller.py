#!/usr/bin/env python3

import signal
import time
import math
import numpy as np
import json

from .odometry_helpers import PlatformStatics, compute_relative_turning_point, compute_turning_radius_yaw_delta
import rospy
import tf_conversions
import tf2_ros

from math import pi as PI
from typing import Optional
from uuid import UUID

from robot_platform.msg import PlatformStatus, MoveRequest
from std_msgs.msg import String, Duration
from geometry_msgs.msg import Point, PoseStamped, Pose, TransformStamped, Point, PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, NavSatFix, NavSatStatus, Imu

from .ros_helpers import ROSNode
from .log_utils import env2log
from .message_utils import parse_response, Request
from .serial_utils import SafeSerialWrapper
from .tf_helpers import *


class WheelController(ROSNode, SafeSerialWrapper):

    def __init__(self):
        ROSNode.__init__(self, 'wheel_controller')
        self._message_counter = 0
        self._motor_distances = [0.0] * PlatformStatics.MOTOR_NUM
        self._total_yaw = 0.0
        self._total_X = 0.0
        self._total_Y = 0.0
        self._last_timestmamp = time.time()
        self._next_request_finish_time = time.time()

        serial_dev = rospy.get_param('~serial_dev')
        serial_baudrate = rospy.get_param('~serial_baudrate')
        SafeSerialWrapper.__init__(self, serial_dev, serial_baudrate)

        self._base_frame_id = rospy.get_param('~base_frame_id')
        self._base_footprint_frame_id = rospy.get_param('~base_footprint_frame_id')
        self._odom_frame_id = rospy.get_param('~odom_frame_id')
        self._laser_frame_id = rospy.get_param('~laser_frame_id')
        self._computed_turning_point_frame_id = rospy.get_param('~computed_turning_point_frame_id')
        self._imu_frame_id = rospy.get_param('~imu_frame_id')
        self._camera_frame_id = rospy.get_param('~camera_frame_id')

        raw_input_topic = rospy.get_param('~raw_input_topic')
        raw_output_topic = rospy.get_param('~raw_output_topic')
        shutdown_command_input_topic = rospy.get_param('~shutdown_command_input_topic')
        rospy.Subscriber(raw_input_topic, String, self._write_raw_data)
        self._raw_log_publisher = rospy.Publisher(raw_output_topic, String)
        rospy.Subscriber(shutdown_command_input_topic, String, self._handle_shutdown_command)

        wheel_positions_input_topic = rospy.get_param('~wheel_positions_input_topic')
        platform_status_output_topic = rospy.get_param('~platform_status_output_topic')
        battery_state_output_topic = rospy.get_param('~battery_state_output_topic')
        gps_state_output_topic = rospy.get_param('~gps_state_output_topic')
        imu_state_output_topic = rospy.get_param('~imu_state_output_topic')
        odometry_output_topic = rospy.get_param('~odometry_output_topic')
        pose_output_topic = rospy.get_param('~pose_output_topic')

        rospy.Subscriber(wheel_positions_input_topic, MoveRequest, self._handle_wheel_inputs)
        self._platform_status_publisher = rospy.Publisher(platform_status_output_topic, PlatformStatus, queue_size=10)
        self._battery_state_publisher = rospy.Publisher(battery_state_output_topic, BatteryState, queue_size=10)
        self._gps_state_publisher = rospy.Publisher(gps_state_output_topic, NavSatFix, queue_size=10)
        self._imu_state_publisher = rospy.Publisher(imu_state_output_topic, Imu, queue_size=10)
        self._odometry_publisher = rospy.Publisher(odometry_output_topic, Odometry, queue_size=10)
        self._pose_publisher = rospy.Publisher(pose_output_topic, PoseStamped, queue_size=10)
        

        rospy.Timer(rospy.Duration(0.001), self._handle_serial)

        self._tf2_broadcaster = tf2_ros.TransformBroadcaster()

        self.spin()

        result = self.write_data('{"move_duration":3,"motor1":{"angle":0.0},"motor2":{"angle":0.0},"motor3":{"angle":0.0},"motor4":{"angle":0.0},"pan":{"angle":0},"tilt":{"angle":0}}')
        if not result:
            self.stop()
    
    def _handle_shutdown_command(self, ros_data:String):
        if os.path.isfile('/shutdown_signal'):
            with open('/shutdown_signal', 'w') as f:
                f.write('true')

    def _handle_serial(self, *_args, **_kwargs):
        raw_data = self.read_data()
        if raw_data:
            self.parse_serial(raw_data)

    def _handle_wheel_inputs(self, raw_data:MoveRequest):
        r = Request.from_MoveRequest(raw_data)
        self._next_request_finish_time = r.move_duration * 0.7 + time.time()
        json_r = r.model_dump_json(exclude_none=True, exclude_unset=True)
        rospy.loginfo(f"requesting: '{json_r}'")
        if not self.write_data(json_r):
            self.stop()
        time.sleep(r.move_duration * 0.7)

    def parse_serial(self, raw_data:String):
        rospy_time_now = rospy.Time.now()
        timestamp = time.time()

        response = parse_response(raw_data)
        if not response:
            return
        self._current_move_duration = response.move_duration

        raw_string = String()
        raw_string.data = raw_data
        self._raw_log_publisher.publish(raw_string)

        platform_status = response.parse_ROS(self._base_frame_id, rospy_time_now)

        self._platform_status_publisher.publish(platform_status)
        self._battery_state_publisher.publish(platform_status.battery)
        self._imu_state_publisher.publish(platform_status.imu)
        if platform_status.gps:
            self._gps_state_publisher.publish(platform_status.gps)

        if self._message_counter == 0:
            rospy.loginfo(f"Battery level: {response.battery.voltage}V")
        self._message_counter = (self._message_counter + 1) % 100

        transforms = [
            # create_static_transform(self._base_footprint_frame_id, self._base_frame_id, 0.0, 0.0, 0.07, 0, 0, 0, rospy_time_now),
            create_static_transform(self._base_frame_id, self._laser_frame_id, 0.13, 0.0, 0.30, 0, 0, math.pi, rospy_time_now),
            create_static_transform(self._base_frame_id, self._camera_frame_id, 0.13, 0.0, 0.5, 0, -response.tilt.angle/2, response.pan.angle, rospy_time_now)
        ]
        mean_distance_delta = sum([m.distance for m in response.motor_list]) / len(response.motor_list)
        computed_turning_point = compute_relative_turning_point(response.motor_list)
        yaw_delta = 0.0
        if computed_turning_point:
            turning_radius, yaw_delta = compute_turning_radius_yaw_delta(computed_turning_point, response.motor_list)
            if abs(mean_distance_delta) > 0:
                self._total_yaw += -yaw_delta if computed_turning_point.y < 0 else yaw_delta

            # transforms.append(create_static_transform(self._base_footprint_frame_id, self._computed_turning_point_frame_id, computed_turning_point.x, computed_turning_point.y, 0, 0, 0, 0, rospy_time_now))
        
        self._total_X += mean_distance_delta * math.cos(self._total_yaw)
        self._total_Y += mean_distance_delta * math.sin(self._total_yaw)

        # if all([m.ready for m in response.motor_list]) and timestamp > self._next_request_finish_time:
        if timestamp > self._next_request_finish_time:
            odometry = Odometry()
            odometry.header.stamp = rospy_time_now
            odometry.header.frame_id = self._base_frame_id
            odometry.child_frame_id = self._odom_frame_id
            odometry.pose.pose.position.x = self._total_X
            odometry.pose.pose.position.y = self._total_Y
            odometry.pose.pose.orientation = get_quaterion_from_rpy(0, 0, self._total_yaw)
            odometry.twist.twist.linear.x = mean_distance_delta / (timestamp - self._last_timestmamp)
            odometry.twist.twist.angular.z = yaw_delta / (timestamp - self._last_timestmamp)

            if odometry.twist.twist.linear.x  == 0 and odometry.twist.twist.angular.z == 0:
                odometry.twist.covariance[0] = 0.01 / 1000
                odometry.twist.covariance[7] = 0.01 / 1000
                odometry.twist.covariance[35] = 0.01 / 1000000
            else:
                odometry.twist.covariance[0] = 0.01
                odometry.twist.covariance[7] = 0.01
                odometry.twist.covariance[35] = 0.01
            self._odometry_publisher.publish(odometry)

            pose_stamped = PoseStamped()
            pose_stamped.header = odometry.header
            pose_stamped.pose = odometry.pose.pose
            self._pose_publisher.publish(pose_stamped)

        # transforms.append(create_static_transform(self._odom_frame_id, self._base_footprint_frame_id, 0, 0, 0, 0, 0, self._total_yaw, rospy_time_now))
        
        # imu_tf2_transform = create_static_transform(self._base_footprint_frame_id, self._imu_frame_id, 0, 0, 0, 0, 0, 0, rospy_time_now)
        # imu_tf2_transform.transform.rotation = platform_status.imu.orientation
        # transforms.append(imu_tf2_transform)

        for c, (m_x, m_y), motor_status in zip([c for c in range(PlatformStatics.MOTOR_NUM)], PlatformStatics.ROBOT_MOTORS_DIMENSIONS, response.motor_list):
            transforms.append(create_static_transform(self._base_frame_id, f"motor{c+1}base", m_x, m_y, 0, 0, 0, 0, rospy_time_now))
            transforms.append(create_static_transform(f"motor{c+1}base", f"motor{c+1}servo", 0, 0, 0, 0, 0, -motor_status.angle, rospy_time_now))
            self._motor_distances[c] += motor_status.distance
            motor_twist = motor_status.distance / PlatformStatics.WHEEL_RADIUS
            transforms.append(create_static_transform(f"motor{c+1}servo", f"motor{c+1}wheel", 0, 0, 0, 0, motor_twist, 0, rospy_time_now))
        
        self._tf2_broadcaster.sendTransform(transforms)
        
        self._last_timestmamp = timestamp


    def _write_raw_data(self, ros_data:String):
        self.write_data(ros_data.data)

def main():
    platform = WheelController()
    signal.signal(signal.SIGINT, platform.stop)
    signal.signal(signal.SIGTERM, platform.stop)


