
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

duration = 1.0

class PathPlatformController(ROSNode):

    def __init__(self):
        ROSNode.__init__(self)
        self._last_platform_status = PlatformStatus()
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


    def _handle_trajectory_update(self, cmd_vel:Twist):
        angle = 0
        move_velocity = 0
    
        if cmd_vel.linear.x > 0:
            move_velocity = max(cmd_vel.linear.x, 0.1)
            angle = cmd_vel.angular.z
        elif cmd_vel.linear.x < 0:
            move_velocity = min(cmd_vel.linear.x, -0.1)
            angle = -cmd_vel.angular.z


        r = create_request(move_velocity, duration, self._last_platform_status, self.__compute_turning_point(angle))
        if r:
            angles_changing = abs(r.motor1.servo.angle - self._last_angles[0]) > 0.025
            angles_changing = angles_changing or abs(r.motor2.servo.angle - self._last_angles[1]) > 0.025
            angles_changing = angles_changing or abs(r.motor3.servo.angle - self._last_angles[2]) > 0.025
            angles_changing = angles_changing or abs(r.motor4.servo.angle - self._last_angles[3]) > 0.025
            if angles_changing:
                r.motor1.velocity = 0.25 * r.motor1.velocity
                r.motor2.velocity = 0.25 * r.motor2.velocity
                r.motor3.velocity = 0.25 * r.motor3.velocity
                r.motor4.velocity = 0.25 * r.motor4.velocity
            self._move_request_publisher.publish(r)
            
            self._last_angles = [
                r.motor1.servo.angle,
                r.motor2.servo.angle,
                r.motor3.servo.angle,
                r.motor4.servo.angle
            ]

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