
import math
import signal

import tf2_ros
from .tf_helpers import *
from .odometry_helpers import *
import rospy
from .ros_helpers import ROSNode

from robot_platform.msg import PlatformStatus, MoveRequest
from geometry_msgs.msg import PoseArray, Twist
from nav_msgs.msg import Odometry

duration = 0.1

class PathPlatformController(ROSNode):

    def __init__(self):
        ROSNode.__init__(self)
        self._last_platform_status = PlatformStatus()

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


    def _handle_trajectory_update(self, cmd_vel:Twist):
        forward = cmd_vel.linear.x
        angle = cmd_vel.angular.z

        move_velocity = forward / duration
        move_duration = duration

        r = create_request(move_velocity, move_duration, self._last_platform_status, self.__compute_turning_point(angle))
        if r:
            self._move_request_publisher.publish(r)

    def _handle_platform_status(self, status:PlatformStatus):
        self._last_platform_status = status

    def __compute_turning_point(self, angle_delta:float) -> Optional[float]:
        turning_point = None
        min_radius = 0.3
        max_radius = 1.0
        tightness_coeff = 4.0
        
        if angle_delta < 0:
            turning_point = Point()
            turning_point.y = max(min_radius + round(max_radius - tightness_coeff*max_radius*angle_delta/math.pi, 1), min_radius)
        elif angle_delta > 0:
            turning_point = Point()
            turning_point.y = -max(min_radius + round(max_radius + tightness_coeff*max_radius*angle_delta/math.pi, 1), min_radius)

        return turning_point


def main():
    platform = PathPlatformController()
    signal.signal(signal.SIGINT, platform.stop)
    signal.signal(signal.SIGTERM, platform.stop)
    platform.start()