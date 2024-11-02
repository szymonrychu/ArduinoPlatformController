
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
SLOW_SPEED = 0.15
MAX_SPEED = 0.3
ROTATION_SPEED = math.pi/0.8
TINY_WAIT_S = 0.1
CAN_MOVE_CONTINOUSLY_CTR_MAX = 5
MAX_TURN_RADIUS = 2.0

REQUEST_DURATION=0.1
MAX_SERVO_ROTATION_ANGLE_WITHIN_REQUEST = PlatformStatics.TURN_VELOCITY * REQUEST_DURATION

class PathPlatformController(ROSNode):


    def __init__(self):
        ROSNode.__init__(self)
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
        abs_turn_radius = max(MAX_TURN_RADIUS - math.pi * abs(angle), PlatformStatics.ROBOT_WIDTH/2 + 0.01)
        turn_radius = abs_turn_radius if cmd_vel.angular.z > 0 else -abs_turn_radius
        turn_radius = turn_radius if move_velocity > 0 else -turn_radius

        moves_slowly = abs_move_velocity == SLOW_SPEED
        abs_turn_delta = abs(angle - self._last_angle)
        tiny_angle_update = abs_turn_delta < TINY_ANGLE_DELTA
        small_angle_update = abs_turn_delta < SMALL_ANGLE_DELTA
        changing_direction = self._same_sign(move_velocity, self._last_velocity)
        turning_through_0_deg = not self._same_sign(angle, self._last_angle)

        self._last_velocity = move_velocity
        self._last_angle = angle

        turning_point = Point()
        turning_point.y = turn_radius

        if small_angle_update and not moves_slowly:
            r = create_request(move_velocity, 1/self._controller_frequency + 0.5, self._last_platform_status, turning_point)
            self.__send_request(r)
            return

        if turning_through_0_deg and not moves_slowly:
            r = create_request(0.0, 1/self._controller_frequency + 0.5, self._last_platform_status, turning_point)
            self.__send_request(r)
            return
        

        
        r1 = create_request(0.0, 1/(2*self._controller_frequency) + 0.5, self._last_platform_status, turning_point)
        self.__send_request(r1)
        time.sleep(1/(2*self._controller_frequency))
        r2 = create_request(move_velocity, 1/(2*self._controller_frequency) + 0.5, self._last_platform_status, turning_point)
        self.__send_request(r2)
        return
        


        # angle = 0
        # move_velocity = 0
    
        # if cmd_vel.linear.x > 0:
        #     move_velocity = min(max(cmd_vel.linear.x, SLOW_SPEED), MAX_SPEED)
        #     angle = cmd_vel.angular.z * 1.04
        # elif cmd_vel.linear.x < 0:
        #     move_velocity = max(min(cmd_vel.linear.x, -SLOW_SPEED), -MAX_SPEED)
        #     angle = cmd_vel.angular.z * 1.04
            

        # abs_angle_delta = abs(angle - self._last_angle)
        # crossing_0_angle = (angle > 0 and self._last_angle < 0) or (angle < 0 and self._last_angle > 0) and abs_angle_delta > SMALL_ANGLE_DELTA
        

        # angle_delta_tiny = abs_angle_delta < TINY_ANGLE_DELTA
        # moves_slowly = abs(move_velocity) == SLOW_SPEED
        # angle_tiny = abs(angle) < TINY_ANGLE_DELTA
        # changes_direction = (move_velocity < 0 and self._last_velocity > 0) or (move_velocity > 0 and self._last_velocity < 0)

        # self._last_angle = angle
        # self._last_velocity = move_velocity


        # turning_point = Point()

        # if angle != 0 and self._controller_frequency != 0:
        #     turning_point.y = move_velocity / (angle * self._controller_frequency)
        
        # r = create_request(move_velocity, 1.2, self._last_platform_status, turning_point)
        # self.__send_request(r)

        # if (angle_delta_tiny or angle_tiny) and not changes_direction:
        #     rospy.loginfo(f"Handling tiny turn without slowdown delta={abs_angle_delta}")
        #     r = create_request(move_velocity, 1.2, self._last_platform_status, turning_point)
        #     self.__send_request(r)
        # else:
        #     rospy.loginfo(f"Handling big turn with full stop and servo readjustment delta={abs_angle_delta}")
        #     r = create_request(move_velocity, 1.2, self._last_platform_status, turning_point)
        #     r_in_place = deepcopy(r)
        #     r_in_place.motor1.velocity = 0
        #     r_in_place.motor2.velocity = 0
        #     r_in_place.motor3.velocity = 0
        #     r_in_place.motor4.velocity = 0
        #     r_in_place.duration = ROTATION_SPEED * self._controller_frequency * (abs_angle_delta/math.pi) # min servo turn duration
        #     self.__send_request(r_in_place)
        #     time.sleep(r_in_place.duration) # wait until servos are fully turned
        #     counter = 0
        #     while self._still_turning(r_in_place) or self._cant_move_continously(angle):
        #         if counter > 75:
        #             break
        #         counter+=1
        #         time.sleep(0.01)
        #     self.__send_request(r) # send move forward request

    def _handle_platform_status(self, status:PlatformStatus):
        self._last_platform_status = status



def main():
    platform = PathPlatformController()
    signal.signal(signal.SIGINT, platform.stop)
    signal.signal(signal.SIGTERM, platform.stop)
    platform.start()