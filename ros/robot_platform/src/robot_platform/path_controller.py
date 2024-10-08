
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

TINY_ANGLE_DELTA = math.pi/36
SMALL_ANGLE_DELTA = math.pi/12
SLOW_SPEED = 0.1
ROTATION_SPEED = math.pi/0.8
TINY_WAIT_S = 0.1
CAN_MOVE_CONTINOUSLY_CTR_MAX = 5

class PathPlatformController(ROSNode):


    def __init__(self):
        ROSNode.__init__(self)
        self._last_platform_status = PlatformStatus()
        self._last_angle = 0
        self.__can_move_continously = 0

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
    
    def _still_turning(self, r:MoveRequest) -> bool:
        status_angles = [
            self._last_platform_status.motor1.servo.angle,
            self._last_platform_status.motor2.servo.angle,
            self._last_platform_status.motor3.servo.angle,
            self._last_platform_status.motor4.servo.angle
        ]
        requested_angles = [
            r.motor1.servo.angle,
            r.motor2.servo.angle,
            r.motor3.servo.angle,
            r.motor4.servo.angle
        ]
        deltas_too_big = [
            abs(s - r) > math.pi/36 for s, r in zip(status_angles, requested_angles)
        ]
        return any(deltas_too_big)
        

    def _can_move_continously(self, angle:float) -> bool:
        return self.__can_move_continously == CAN_MOVE_CONTINOUSLY_CTR_MAX or abs(angle) < math.pi/36

    def _handle_trajectory_update(self, cmd_vel:Twist):
        angle = 0
        move_velocity = 0
    
        if cmd_vel.linear.x > 0:
            move_velocity = max(cmd_vel.linear.x, SLOW_SPEED)
            angle = cmd_vel.angular.z * 1.04
        elif cmd_vel.linear.x < 0:
            move_velocity = min(cmd_vel.linear.x, -SLOW_SPEED)
            angle = cmd_vel.angular.z * 1.04
            

        abs_angle_delta = abs(angle - self._last_angle)
        crossing_0_angle = (angle > 0 and self._last_angle < 0) or (angle < 0 and self._last_angle > 0) and abs_angle_delta > SMALL_ANGLE_DELTA
        

        angle_delta_tiny = abs_angle_delta < TINY_ANGLE_DELTA
        moves_slowly = abs(move_velocity) == SLOW_SPEED
        angle_tiny = abs(angle) < TINY_ANGLE_DELTA
        changes_direction = (move_velocity < 0 and self._last_velocity > 0) or (move_velocity > 0 and self._last_velocity < 0)

        self._last_angle = angle
        self._last_velocity = move_velocity


        turning_point = Point()

        if angle != 0 and self._controller_frequency != 0:
            turning_point.y = move_velocity / (angle * self._controller_frequency)

        if (angle_delta_tiny or angle_tiny or self._can_move_continously(angle)) and not changes_direction:
            rospy.loginfo(f"Handling tiny turn without slowdown delta={abs_angle_delta}")
            r = create_request(move_velocity, 1.2, self._last_platform_status, turning_point)
            self.__send_request(r)
        else:
            rospy.loginfo(f"Handling big turn with full stop and servo readjustment delta={abs_angle_delta}")
            r = create_request(move_velocity, 1.2, self._last_platform_status, turning_point)
            r_in_place = deepcopy(r)
            r_in_place.motor1.velocity = 0
            r_in_place.motor2.velocity = 0
            r_in_place.motor3.velocity = 0
            r_in_place.motor4.velocity = 0
            r_in_place.duration = ROTATION_SPEED * self._controller_frequency * (abs_angle_delta/math.pi) # min servo turn duration
            self.__send_request(r_in_place)
            time.sleep(r_in_place.duration) # wait until servos are fully turned
            counter = 0
            while self._still_turning(r_in_place) or self._cant_move_continously(angle):
                if counter > 75:
                    break
                counter+=1
                time.sleep(0.01)
            self.__send_request(r) # send move forward request

    def _handle_platform_status(self, status:PlatformStatus):
        if compute_relative_turning_point([
                status.motor1.servo, status.motor2.servo, status.motor3.servo, status.motor4.servo
            ]) is not None:
            if self.__can_move_continously < CAN_MOVE_CONTINOUSLY_CTR_MAX:
                self.__can_move_continously += 1
        else:
            self.__can_move_continously = 0
        self._last_platform_status = status



def main():
    platform = PathPlatformController()
    signal.signal(signal.SIGINT, platform.stop)
    signal.signal(signal.SIGTERM, platform.stop)
    platform.start()