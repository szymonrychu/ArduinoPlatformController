
import math
import signal

import tf2_ros
from .tf_helpers import *
from .odometry_helpers import *
import rospy
from .ros_helpers import ROSNode

from robot_platform.msg import MoveRequest
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

duration = 0.1

class PathPlatformController(ROSNode):

    def __init__(self):
        ROSNode.__init__(self)
        self._busy = False

        move_request_output_topic = rospy.get_param('~move_request_output_topic')
        self._move_request_publisher = rospy.Publisher(move_request_output_topic, MoveRequest)
        rospy.Subscriber('/cmd_vel', Twist, self._handle_trajectory_update)

        # rospy.Timer(rospy.Duration(duration), self._send_request)
        self.spin()

    def _handle_trajectory_update(self, cmd_vel:Twist):
        forward = cmd_vel.linear.x
        angle = cmd_vel.angular.z

        move_velocity = forward / duration
        move_duration = duration

        r = create_request(move_velocity, move_duration, self._last_platform_status, self.__compute_turning_point(angle))
        self._move_request_publisher.publish(r)


def main():
    platform = PathPlatformController()
    signal.signal(signal.SIGINT, platform.stop)
    signal.signal(signal.SIGTERM, platform.stop)
    platform.start()