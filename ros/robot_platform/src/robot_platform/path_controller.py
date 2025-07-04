
import math
import signal
import time

import tf2_ros
from .tf_helpers import *
from .odometry_helpers import *
import rospy
from .ros_helpers import ROSNode
from copy import deepcopy

from robot_platform.msg import PlatformStatus, MoveRequest
from geometry_msgs.msg import PoseArray, Twist, Point
from nav_msgs.msg import Odometry

TINY_ANGLE_DELTA = math.pi/18
SMALL_ANGLE_DELTA = math.pi/12
SLOW_SPEED = 0.1
MAX_SPEED = 0.4

class PathPlatformController(ROSNode):


    def __init__(self):
        ROSNode.__init__(self, 'path_controller')
        self._last_platform_status = PlatformStatus()
        self._last_angle = 0

        self._last_odometry = Odometry()

        self._last_pose_array = PoseArray()
        self._last_angle = 0.0
        self._last_velocity = 0.0

        move_request_output_topic = rospy.get_param('~move_request_output_topic')
        platform_status_input_topic = rospy.get_param('~platform_status_input_topic')
        self._controller_frequency = rospy.get_param('~controller_frequency')
        
        self._move_request_publisher = rospy.Publisher(move_request_output_topic, MoveRequest)
        rospy.Subscriber(platform_status_input_topic, PlatformStatus, self._handle_platform_status)
        rospy.Subscriber('/cmd_vel', Twist, self._handle_trajectory_update)
        self.spin()

    def __send_request(self, r):
        if r:
            self._move_request_publisher.publish(r)

    def _same_sign(self, x:float, y:float):
        both_are_positive = x >= 0 and y >= 0
        both_are_negative = x < 0 and y < 0
        return both_are_positive or both_are_negative

    def _handle_trajectory_update(self, cmd_vel:Twist):
        abs_move_velocity = min(max(abs(cmd_vel.linear.x), SLOW_SPEED), MAX_SPEED)

        move_velocity = abs_move_velocity if cmd_vel.linear.x > 0 else -abs_move_velocity
        angle = cmd_vel.angular.z

        turning_point = Point()
        if angle != 0:
            abs_turn_radius = move_velocity / abs(angle)* self._controller_frequency
            turn_radius = abs_turn_radius if cmd_vel.angular.z > 0 else -abs_turn_radius
            turning_point.y = turn_radius

        self._last_velocity = move_velocity
        self._last_angle = angle

        for r in create_requests(move_velocity, 1/self._controller_frequency, self._last_platform_status, turning_point):
            self.__send_request(r)

    def _handle_platform_status(self, status:PlatformStatus):
        self._last_platform_status = status



def main():
    platform = PathPlatformController()
    signal.signal(signal.SIGINT, platform.stop)
    signal.signal(signal.SIGTERM, platform.stop)
