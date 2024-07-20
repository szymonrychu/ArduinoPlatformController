
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
SMALL_ANGLE_DELTA = 0.15
TINY_ANGLE_DELTA = 0.075
SLOW_MOVE = 0.1
SLOW_DOWN_FACTOR = 2.0

class PathPlatformController(ROSNode):

    def __init__(self):
        ROSNode.__init__(self)
        self._last_platform_status = PlatformStatus()
        self._last_angle = 0
        self._last_direction_forward = False
        self._last_direction_backward = False
        self._can_move_continously = True
        self._zero_velocity_commands_num = 0

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


    def _handle_trajectory_update(self, cmd_vel:Twist):
        angle = 0
        move_velocity = 0
    
        if cmd_vel.linear.x > 0:
            move_velocity = max(cmd_vel.linear.x, 0.2)
            angle = cmd_vel.angular.z
            self._last_direction_forward = True
            self._last_direction_backward = False
            self._zero_velocity_commands_num = 0
        elif cmd_vel.linear.x < 0:
            move_velocity = min(cmd_vel.linear.x, -0.2)
            angle = -cmd_vel.angular.z
            self._last_direction_forward = False
            self._last_direction_backward = True
            self._zero_velocity_commands_num = 0
        elif self._last_direction_forward and self._zero_velocity_commands_num < 5:
            move_velocity = 0.2
            angle = cmd_vel.angular.z
            self._zero_velocity_commands_num += 1
        elif self._last_direction_backward and self._zero_velocity_commands_num < 5:
            move_velocity = -0.2
            angle = -cmd_vel.angular.z
            self._zero_velocity_commands_num += 1
        else:
            return
            

        abs_angle_delta = abs(angle - self._last_angle)
        crossing_0_angle = (angle > 0 and self._last_angle < 0) or (angle < 0 and self._last_angle > 0) and abs_angle_delta > SMALL_ANGLE_DELTA
        self._last_angle = angle


        if abs_angle_delta > TINY_ANGLE_DELTA or crossing_0_angle or abs(move_velocity) < 0.25 or not self._can_move_continously: # it's a big turn, we need to stop entirely
            if abs(angle) < TINY_ANGLE_DELTA: # after turning we will go relatively straight, we can go with full speed
                rospy.loginfo(f"Handling big turn with full stop and servo readjustment delta={abs_angle_delta}")
                r = create_request(move_velocity, duration, self._last_platform_status, self.__compute_turning_point(angle))
                r_in_place = deepcopy(r)
                r_in_place.motor1.velocity = 0
                r_in_place.motor2.velocity = 0
                r_in_place.motor3.velocity = 0
                r_in_place.motor4.velocity = 0
                self.__send_request(r_in_place)
                r_in_place.duration = abs_angle_delta/PlatformStatics.TURN_VELOCITY # min servo turn duration
                time.sleep(r_in_place.duration*1.2) # wait until servos are fully turned
                self.__send_request(r) # send move forward request
            else: # after turning servos, we will turn, so we have to be slower
                rospy.loginfo(f"Handling big turn with full stop and servo readjustment delta={abs_angle_delta}")
                r = create_request(move_velocity/SLOW_DOWN_FACTOR, duration, self._last_platform_status, self.__compute_turning_point(angle))
                r_in_place = deepcopy(r)
                r_in_place.motor1.velocity = 0
                r_in_place.motor2.velocity = 0
                r_in_place.motor3.velocity = 0
                r_in_place.motor4.velocity = 0
                self.__send_request(r_in_place)
                r_in_place.duration = abs_angle_delta/PlatformStatics.TURN_VELOCITY # min servo turn duration
                time.sleep(r_in_place.duration*1.2) # wait until servos are fully turned
                self.__send_request(r) # send move forward request
        else: # it's a small turn, we can do turning and moving at the same time
            if abs(angle) < TINY_ANGLE_DELTA or abs(move_velocity) < 0.25: # it's just readjustment in going forward, we can avoid slowing down
                rospy.loginfo(f"Handling tiny turn without slowdown delta={abs_angle_delta}")
                r = create_request(move_velocity, duration, self._last_platform_status, self.__compute_turning_point(angle))
                self.__send_request(r)
            else: # we are not going straight, we should slow down
                rospy.loginfo(f"Handling small turn with slowdown delta={abs_angle_delta}")
                r = create_request(move_velocity/SLOW_DOWN_FACTOR, duration, self._last_platform_status, self.__compute_turning_point(angle))
                self.__send_request(r)


    def _handle_platform_status(self, status:PlatformStatus):
        self._can_move_continously = compute_relative_turning_point([
            status.motor1, status.motor2, status.motor3, status.motor4
        ])
        self._last_platform_status = status

    def __compute_turning_point(self, angle_delta:float) -> Optional[float]:

        if angle_delta > 0.1:
            turn_radius = 1.5 * (1.0-abs(angle_delta/0.3)) + 0.3
        elif angle_delta < 0.1:
            turn_radius = -1.5 * (1.0-abs(angle_delta/0.3)) - 0.3
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