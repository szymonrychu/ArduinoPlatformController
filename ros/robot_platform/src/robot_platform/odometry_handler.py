#!/usr/bin/env python3

import signal
import time
import math
import numpy as np

import rospy
import tf_conversions
import tf2_ros

# from math import pi as PI
# from typing import Optional
# from uuid import UUID

from .ros_helpers import ROSNode
from robot_platform.msg import PlatformStatus
from geometry_msgs.msg import Pose, PoseStamped, Point

from .log_utils import env2log
from .tf_helpers import *
from .odometry_helpers import PlatformStatics

class OdometryHandlerROSNode(ROSNode):

    def __init__(self):
        ROSNode.__init__(self)
        self.__current_Pose = Pose()

        platform_status_input_topic = rospy.get_param('~platform_status_input_topic')
        rospy.Subscriber(platform_status_input_topic, PlatformStatus, self._handle_platform_status)

        odom_state_output_topic = rospy.get_param('~odom_state_output_topic')
        self._odom_state_publisher = rospy.Publisher(odom_state_output_topic, PoseStamped)

    def _handle_platform_status(self, status:PlatformStatus):
        motors = [
            status.motor1,
            status.motor2,
            status.motor3,
            status.motor4,
        ]
        mean_distance = sum([m.distance for m in motors])/len(motors)

        turning_points = []

        for i1 in range(PlatformStatics.MOTOR_NUM):
            for i2 in range(PlatformStatics.MOTOR_NUM):
                if i2 >= i1:
                    continue

                yaw_A, (XA, YA) = motors[i1].servo.angle, PlatformStatics.ROBOT_MOTORS_DIMENSIONS[i1]
                if i1 % 2 == 1:
                    yaw_A = -yaw_A
                
                yaw_B, (XB, YB) = motors[i2].servo.angle, PlatformStatics.ROBOT_MOTORS_DIMENSIONS[i2]
                if i2 % 2 == 1:
                    yaw_B = -yaw_B
                
                if abs(yaw_A - yaw_B) < PlatformStatics.MIN_ANGLE_DIFF:
                    continue

                p = Point()
                p.y = ( YA * math.tan(math.pi/2 - yaw_A) + YB * math.tan(math.pi/2 - yaw_B) + XB - XA ) / (math.tan(math.pi/2 - yaw_A) - math.tan(math.pi/2 - yaw_B))
                p.x = XA - (YA - p.y) * math.tan(math.pi/2 - yaw_A)

                turning_points.append(p)
        
        relative_turning_point = None
        if turning_points:
            relative_turning_point = Point()
            relative_turning_point.x = sum([p.x for p in turning_points])/len(turning_points)
            relative_turning_point.y = sum([p.y for p in turning_points])/len(turning_points)

            turning_radius = math.sqrt(relative_turning_point.x**2 + relative_turning_point.y**2)
            full_circle_distance = 2 * math.pi * turning_radius
            angle_radians_delta = full_circle_distance / mean_distance

            self._current_YAW += angle_radians_delta

        # self.__current_Pose.position.x += motor_status.distance * math.cos(self._current_YAW)
        # self.__current_Pose.position.y += motor_status.distance * math.sin(self._current_YAW)
        # self.__current_Pose.orientation = get_quaterion_from_rpy(0.0, 0.0, self._current_YAW)

def main():
    platform = OdometryHandlerROSNode()
    signal.signal(signal.SIGINT, platform.stop)
    signal.signal(signal.SIGTERM, platform.stop)
    platform.start()


