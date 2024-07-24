
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
TINY_ANGLE_DELTA = 0.05
SMALL_ANGLE_DELTA = 0.2
SLOW_SPEED = 0.2
ROTATION_SPEED = math.pi/(0.18*3)

class PathPlatformController(ROSNode):

    def __init__(self):
        ROSNode.__init__(self)
        self._last_platform_status = PlatformStatus()
        self._last_angle = 0
        self._can_move_continously = True

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

    def __send_request(self, r):
        if r:
            self._move_request_publisher.publish(r)

    def __can_move_continously(self, angle:float) -> bool:
        return self._can_move_continously or abs(angle) < SMALL_ANGLE_DELTA

    def _handle_trajectory_update(self, cmd_vel:Twist):
        angle = 0
        move_velocity = 0
    
        if cmd_vel.linear.x > 0:
            move_velocity = max(cmd_vel.linear.x, 0.2)
            angle = cmd_vel.angular.z
        elif cmd_vel.linear.x < 0:
            move_velocity = min(cmd_vel.linear.x, -0.2)
            angle = -cmd_vel.angular.z
            

        abs_angle_delta = abs(angle - self._last_angle)
        crossing_0_angle = (angle > 0 and self._last_angle < 0) or (angle < 0 and self._last_angle > 0) and abs_angle_delta > SMALL_ANGLE_DELTA
        self._last_angle = angle
        

        angle_delta_tiny = abs_angle_delta < TINY_ANGLE_DELTA
        moves_slowly = abs(move_velocity) < SLOW_SPEED
        angle_tiny = abs(angle) < TINY_ANGLE_DELTA

        if angle_tiny and (not moves_slowly) and self.__can_move_continously(angle):
            rospy.loginfo(f"Handling tiny turn without slowdown delta={abs_angle_delta}")
            r = create_request(move_velocity, duration, self._last_platform_status, self.__compute_turning_point(angle))
            self.__send_request(r)
        else:
            rospy.loginfo(f"Handling big turn with full stop and servo readjustment delta={abs_angle_delta}")
            r = create_request(move_velocity, duration, self._last_platform_status, self.__compute_turning_point(angle))
            r_in_place = deepcopy(r)
            r_in_place.motor1.velocity = 0
            r_in_place.motor2.velocity = 0
            r_in_place.motor3.velocity = 0
            r_in_place.motor4.velocity = 0
            self.__send_request(r_in_place)
            r_in_place.duration = ROTATION_SPEED * (abs_angle_delta/math.pi) # min servo turn duration
            time.sleep(r_in_place.duration) # wait until servos are fully turned
            while not self.__can_move_continously(angle):
                time.sleep(0.01)
            self.__send_request(r) # send move forward request

    def _handle_platform_status(self, status:PlatformStatus):
        self._can_move_continously = compute_relative_turning_point([
            status.motor1.servo, status.motor2.servo, status.motor3.servo, status.motor4.servo
        ]) is not None
        self._last_platform_status = status

    def __compute_turning_point(self, angle_delta:float) -> Optional[float]:

        if angle_delta > 0.1:
            turn_radius = 5.0 * (1.0-abs(angle_delta/0.3)) + 0.3
        elif angle_delta < 0.1:
            turn_radius = -5.0 * (1.0-abs(angle_delta/0.3)) - 0.3
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