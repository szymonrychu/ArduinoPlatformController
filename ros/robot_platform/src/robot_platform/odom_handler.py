
from .message_utils import StatusResponse, Request
from .tf_helpers import *

from typing import List

import math
from geometry_msgs.msg import Pose

ROBOT_LENGTH = 0.350
ROBOT_WIDTH = 0.248

MIN_TURN_COEF = 0.0

M1_IN_PLACE_TURN = -1.0
M1_X = -(ROBOT_WIDTH/2) 
M1_Y = -(ROBOT_LENGTH/2)

M2_IN_PLACE_TURN = -1.0
M2_X = (ROBOT_WIDTH/2) 
M2_Y = -(ROBOT_LENGTH/2)

M3_IN_PLACE_TURN = -1.0
M3_X = (ROBOT_WIDTH/2) 
M3_Y = (ROBOT_LENGTH/2)

M4_IN_PLACE_TURN = -1.0
M4_X = -(ROBOT_WIDTH/2) 
M4_Y = (ROBOT_LENGTH/2)


def limit_angle(angle:float) -> float:
    if angle > math.pi/2:
        return angle - math.pi
    if angle < -math.pi/2:
        return angle + math.pi
    return angle

def turn_around_XY(turn_angle:float, servo_rotation_duration:float, movement_duration:float, x:float = 0, y:float = 0) -> List[Request]:
    moves = []

    m1_y, m1_x = M1_Y - y, M1_X - x
    m2_y, m2_x = M2_Y - y, M2_X - x
    m3_y, m3_x = M3_Y - y, M3_X - x
    m4_y, m4_x = M4_Y - y, M4_X - x

    servo_move = Request()
    servo_move.motor1.angle = M1_IN_PLACE_TURN * math.atan2(m1_y, m1_x)
    servo_move.motor2.angle = M2_IN_PLACE_TURN * math.atan2(m2_y, m2_x)
    servo_move.motor3.angle = M3_IN_PLACE_TURN * math.atan2(m3_y, m3_x)
    servo_move.motor4.angle = M4_IN_PLACE_TURN * math.atan2(m4_y, m4_x)
    servo_move.move_duration = servo_rotation_duration
    moves.append(servo_move)

    m1_radius = math.sqrt(m1_x * m1_x + m1_y * m1_y)
    m2_radius = math.sqrt(m2_x * m2_x + m2_y * m2_y)
    m3_radius = math.sqrt(m3_x * m3_x + m3_y * m3_y)
    m4_radius = math.sqrt(m4_x * m4_x + m4_y * m4_y)

    mean_radius = (m1_radius + m2_radius + m3_radius + m4_radius) / 4.0

    m1_coeficient = math.max(m1_radius/mean_radius, MIN_TURN_COEF)
    m2_coeficient = math.max(m2_radius/mean_radius, MIN_TURN_COEF)
    m3_coeficient = math.max(m3_radius/mean_radius, MIN_TURN_COEF)
    m4_coeficient = math.max(m4_radius/mean_radius, MIN_TURN_COEF)

    m1_coeficient = m1_coeficient if x < M1_X else -m1_coeficient
    m2_coeficient = m2_coeficient if x > M2_X else -m2_coeficient
    m3_coeficient = m3_coeficient if x > M3_X else -m3_coeficient
    m4_coeficient = m4_coeficient if x < M4_X else -m4_coeficient

    distance = turn_angle * mean_radius # (angle/2pi) * 2piR = angle * R
    velocity = turn_distance / movement_duration

    motor_move = Request()
    motor_move.motor1.velocity = m1coef * velocity
    motor_move.motor2.velocity = m2coef * velocity
    motor_move.motor3.velocity = m3coef * velocity
    motor_move.motor4.velocity = m4coef * velocity
    motor_move.move_duration = movement_duration
    moves.append(motor_move)

    return moves




class OdomHandler():

    def handle_motor_updates(self, response:StatusResponse) -> Pose:
        p = Pose()
        p.orientation = get_quaterion_from_rpy(0, 0, 0)
        return p

    def request_move(self, goal_pose:PoseStamped) -> Request:
        r = Request()
        return r