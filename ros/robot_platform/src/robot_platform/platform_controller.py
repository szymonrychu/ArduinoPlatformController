#!/usr/bin/env python3

import signal
import time
import math

from .odometry_helpers import PlatformStatics, create_requests, compute_relative_turning_point, compute_turning_radius_yaw_delta
import rospy
import tf2_ros

from math import pi as PI
from typing import List, Optional
from uuid import UUID
from threading import Lock

from robot_platform.msg import PlatformStatus, PlatformRequest
from std_msgs.msg import String, Duration
from geometry_msgs.msg import Twist, PoseStamped, Pose, TransformStamped, Point, PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, NavSatFix, NavSatStatus, Imu

from .ros_helpers import ROSNode
from .log_utils import env2log
from .models import parse_response, Request
from .serial_utils import SafeSerialWrapper
from .tf_helpers import *


class WheelController(SafeSerialWrapper):

    def __init__(self):
        rospy.init_node('wheel_controller', log_level=rospy.DEBUG)
        rospy.loginfo('Started')

        serial_dev = rospy.get_param('~serial_dev')
        serial_baudrate = rospy.get_param('~serial_baudrate')
        SafeSerialWrapper.__init__(self, serial_dev, serial_baudrate)
        rospy.loginfo('Initialised serial')

        # frames for TF
        self._base_frame_id = str(rospy.get_param('~base_frame_id'))
        self._base_footprint_frame_id = str(rospy.get_param('~base_footprint_frame_id'))
        self._odom_frame_id = str(rospy.get_param('~odom_frame_id'))
        self._laser_frame_id = str(rospy.get_param('~laser_frame_id'))
        self._computed_turning_point_frame_id = str(rospy.get_param('~computed_turning_point_frame_id'))
        self._imu_frame_id = str(rospy.get_param('~imu_frame_id'))
        self._camera_frame_id = str(rospy.get_param('~camera_frame_id'))

        # input topics
        cmd_vel_input_topic = rospy.get_param('~cmd_vel_input_topic')
        wheel_positions_input_topic = rospy.get_param('~wheel_positions_input_topic')
        # shutdown_command_input_topic = rospy.get_param('~shutdown_command_input_topic')
        # rospy.Subscriber(shutdown_command_input_topic, String, self._handle_shutdown_command)

        # output topics
        platform_status_output_topic = rospy.get_param('~platform_status_output_topic')
        battery_state_output_topic = rospy.get_param('~battery_state_output_topic')
        gps_state_output_topic = rospy.get_param('~gps_state_output_topic')
        imu_state_output_topic = rospy.get_param('~imu_state_output_topic')
        odometry_output_topic = rospy.get_param('~odometry_output_topic')
        pose_output_topic = rospy.get_param('~pose_output_topic')

        self._controller_frequency = int(rospy.get_param('~controller_frequency')) # type: ignore

        rospy.loginfo('Loaded params')
        
        self._motor_distances = [0.0] * PlatformStatics.MOTOR_NUM
        self._last_platform_status = None
        
        self._last_cmd_vel_lock = Lock()
        self._last_cmd_vel = None

        rospy.spin()
        
        rospy.Subscriber(wheel_positions_input_topic, PlatformRequest, self._handle_wheel_inputs)
        rospy.Subscriber(cmd_vel_input_topic, Twist, self._handle_cmdvel)
        self._platform_status_publisher = rospy.Publisher(platform_status_output_topic, PlatformStatus, queue_size=10)
        self._battery_state_publisher = rospy.Publisher(battery_state_output_topic, BatteryState, queue_size=10)
        self._gps_state_publisher = rospy.Publisher(gps_state_output_topic, NavSatFix, queue_size=10)
        self._imu_state_publisher = rospy.Publisher(imu_state_output_topic, Imu, queue_size=10)
        self._odometry_publisher = rospy.Publisher(odometry_output_topic, Odometry, queue_size=10)
        self._pose_publisher = rospy.Publisher(pose_output_topic, PoseStamped, queue_size=10)

        # init ROS with tf
        self._tf2_broadcaster = tf2_ros.TransformBroadcaster()

        rospy.loginfo('Initialised')

        # zero robot actuators
        result = self.write_requests(create_requests(3, PlatformStatus()))
        if not result:
            rospy.signal_shutdown(reason="Couldn't write requests!")

        rospy.loginfo('Primed')

    def _handle_wheel_inputs(self, raw_data:PlatformRequest):
        """
        Function handling incoming PlatformRequests

        Args:
            raw_data (PlatformRequest): input provided by rospy with data incoming from external Publisher
        """        
        r = Request.from_ROS_PlatformRequest(raw_data)
        json_r = r.model_dump_json(exclude_none=True, exclude_unset=True)
        rospy.loginfo(f"requesting: '{json_r}'")
        if not self.write_data(json_r):
            self.stop()

    def write_requests(self, requests:List[Request]) -> bool:
        result = []
        for r in requests:
            r_json = r.model_dump_json(exclude_none=True, exclude_unset=True)
            rospy.loginfo(f"Requesting: '{r_json}'")
            result.append(self.write_data(r_json))
        return all(result)
    
    def _handle_cmdvel(self, ros_data:Twist):
        with self._last_cmd_vel_lock:
            self._last_cmd_vel = ros_data

    def _handle_serial(self, *_args, **_kwargs):
        raw_data = self.read_data()
        if not raw_data:
            rospy.logerr('Can\'t read data')
            return

        response = parse_response(raw_data)
        if not response:
            rospy.logerr(f"Couldn't parse data '{raw_data}'")
            return

        if self._last_cmd_vel:
            with self._last_cmd_vel_lock:
                abs_move_velocity = min(max(abs(self._last_cmd_vel.linear.x), PlatformStatics.SLOW_SPEED), PlatformStatics.MAX_SPEED)
                move_velocity = abs_move_velocity if self._last_cmd_vel.linear.x > 0 else -abs_move_velocity
                angle = self._last_cmd_vel.angular.z

                turning_point = None
                if angle != 0:
                    turning_point = Point()
                    abs_turn_radius = move_velocity / abs(angle)* self._controller_frequency
                    turn_radius = abs_turn_radius if self._last_cmd_vel.angular.z > 0 else -abs_turn_radius
                    turning_point.y = turn_radius
                
                if self._last_platform_status:
                    self.write_requests(create_requests(1/self._controller_frequency, self._last_platform_status, velocity=move_velocity, turning_point=turning_point))

        rospy_time_now = rospy.Time.now()
        timestamp = time.time()

        self._last_platform_status = response.to_ROS_PlatformStatus(self._base_frame_id, rospy_time_now)
        self._platform_status_publisher.publish(self._last_platform_status)
        self._battery_state_publisher.publish(self._last_platform_status.battery)
        self._imu_state_publisher.publish(self._last_platform_status.imu)
        if self._last_platform_status.gps:
            self._gps_state_publisher.publish(self._last_platform_status.gps)

        rospy.loginfo_throttle(60, f"Battery level: {response.battery.voltage}V")

        mean_distance_delta = sum([m.distance for m in response.motor_list]) / len(response.motor_list)
        computed_turning_point = compute_relative_turning_point(response.servo_list)
        yaw_delta = 0
        if computed_turning_point:
            _, yaw_delta = compute_turning_radius_yaw_delta(computed_turning_point, response.motor_list)
            self._total_yaw += -yaw_delta if computed_turning_point.y < 0 else yaw_delta

        self._total_X += mean_distance_delta * math.cos(self._total_yaw)
        self._total_Y += mean_distance_delta * math.sin(self._total_yaw)

        if response.move_uuid == None:
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
                odometry.twist.covariance[0] = 0.01 / 1000              # type: ignore
                odometry.twist.covariance[7] = 0.01 / 1000              # type: ignore
                odometry.twist.covariance[35] = 0.01 / 1000000          # type: ignore
            else:
                odometry.twist.covariance[0] = 0.01                     # type: ignore
                odometry.twist.covariance[7] = 0.01                     # type: ignore
                odometry.twist.covariance[35] = 0.01                    # type: ignore
            self._odometry_publisher.publish(odometry)

            pose_stamped = PoseStamped()
            pose_stamped.header = odometry.header
            pose_stamped.pose = odometry.pose.pose
            self._pose_publisher.publish(pose_stamped)
        else:
            rospy.loginfo(f"Move: {response.move_uuid}")

        transforms = [
            create_static_transform(self._base_frame_id, self._laser_frame_id, 0.13, 0.0, 0.30, 0, 0, math.pi, rospy_time_now),
            create_static_transform(self._base_frame_id, self._camera_frame_id, 0.13, 0.0, 0.5, 0, -response.tilt.angle/2, response.pan.angle, rospy_time_now)
        ]
        for c, (m_x, m_y), motor, servo in zip([c for c in range(PlatformStatics.MOTOR_NUM)], PlatformStatics.ROBOT_MOTORS_DIMENSIONS, response.motor_list, response.servo_list):
            transforms.append(create_static_transform(self._base_frame_id, f"motor{c+1}base", m_x, m_y, 0, 0, 0, 0, rospy_time_now))
            transforms.append(create_static_transform(f"motor{c+1}base", f"motor{c+1}servo", 0, 0, 0, 0, 0, -servo.angle, rospy_time_now))
            transforms.append(motor.to_ROS_TF2(f"motor{c+1}servo", f"motor{c+1}wheel", PlatformStatics.WHEEL_RADIUS, timestamp=rospy_time_now))
        
        self._tf2_broadcaster.sendTransform(transforms)
        
        self._last_timestmamp = timestamp

    def stop(self, *args, **kwargs):
        rospy.signal_shutdown(reason="Stopping gracefully!")


def main():
    platform = WheelController()
    signal.signal(signal.SIGINT, platform.stop)
    signal.signal(signal.SIGTERM, platform.stop)


