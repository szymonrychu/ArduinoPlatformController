
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
from geometry_msgs.msg import PoseArray, Twist
from nav_msgs.msg import Odometry

duration = 1.0
SMALL_ANGLE_DELTA = 0.5
SLOW_MOVE = 0.1

class PathPlatformController(ROSNode):

    def __init__(self):
        ROSNode.__init__(self)
        self._last_platform_status = PlatformStatus()
        self._last_angle = 0
        self._last_angles = [
            0.0, 0.0, 0.0, 0.0
        ]

        self._last_odometry = Odometry()

        self._last_pose_array = PoseArray()
        self._pose_counter = 1
        self._last_angle = 0.0

        move_request_output_topic = rospy.get_param('~move_request_output_topic')
        platform_status_input_topic = rospy.get_param('~platform_status_input_topic')
        
        self._move_request_publisher = rospy.Publisher(move_request_output_topic, MoveRequest)
        rospy.Subscriber(platform_status_input_topic, PlatformStatus, self._handle_platform_status)
        rospy.Subscriber('/cmd_vel', Twist, self._handle_trajectory_update)

        self.spin()

    def __send_request(self, r:MoveRequest|None):
        if r:
            self._move_request_publisher.publish(r)



    def _handle_trajectory_update(self, cmd_vel:Twist):
        angle = 0
        move_velocity = 0
    
        if cmd_vel.linear.x > 0:
            move_velocity = max(cmd_vel.linear.x, 0.1)
            angle = cmd_vel.angular.z
        elif cmd_vel.linear.x < 0:
            move_velocity = min(cmd_vel.linear.x, -0.1)
            angle = -cmd_vel.angular.z

        abs_angle_delta = abs(angle - self._last_angle)
        self._last_angle = angle

        if abs_angle_delta < SMALL_ANGLE_DELTA: # it's ok to turn continously, just slow down
            r = create_request(move_velocity/2.0, duration, self._last_platform_status, self.__compute_turning_point(angle))
            self.__send_request(r)
        else: # it's a big turn, we need to stop entirely
            r = create_request(move_velocity, duration, self._last_platform_status, self.__compute_turning_point(angle))
            r_in_place = deepcopy(r)
            r_in_place.motor1.velocity = 0
            r_in_place.motor2.velocity = 0
            r_in_place.motor3.velocity = 0
            r_in_place.motor4.velocity = 0
            self.__send_request(r_in_place)
            r_in_place.duration = abs_angle_delta/PlatformStatics.TURN_VELOCITY # min servo turn duration
            time.sleep(r_in_place.duration) # wait until servos are fully turned

            self.__send_request(r) # send move forward request

    def _handle_platform_status(self, status:PlatformStatus):
        self._last_platform_status = status

    def __compute_turning_point(self, angle_delta:float) -> Optional[float]:

        if angle_delta > 0.1:
            turn_radius = 2.0 * (1.0-abs(angle_delta/0.3)) + 0.3
        elif angle_delta < 0.1:
            turn_radius = -2.0 * (1.0-abs(angle_delta/0.3)) - 0.3
        else:
            turn_radius = 0

        turning_point = Point()
        turning_point.y = turn_radius

        return turning_point


def main():
    platform = PathPlatformController()
    signal.signal(signal.SIGINT, platform.stop)
    signal.signal(signal.SIGTERM, platform.stop)
    platform.start()