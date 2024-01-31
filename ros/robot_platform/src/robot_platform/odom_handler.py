
from .message_utils import StatusResponse, Request
from .tf_helpers import *

from typing import List, Generator

import math
from geometry_msgs.msg import Pose, Point, PointStamped

ROBOT_LENGTH = 0.350
ROBOT_WIDTH = 0.248

MIN_TURN_COEF = 0.0

# LEFT BACK
M1_IN_PLACE_TURN = -1.0
M1_X = -(ROBOT_WIDTH/2) 
M1_Y = (ROBOT_LENGTH/2)

# RIGHT BACK
M2_IN_PLACE_TURN = -1.0
M2_X = -(ROBOT_WIDTH/2) 
M2_Y = -(ROBOT_LENGTH/2)

# RIGHT FRONT
M3_IN_PLACE_TURN = -1.0
M3_X = (ROBOT_WIDTH/2) 
M3_Y = -(ROBOT_LENGTH/2)

# LEFT FRONT
M4_IN_PLACE_TURN = -1.0
M4_X = (ROBOT_WIDTH/2) 
M4_Y = (ROBOT_LENGTH/2)

ROBOT_MOTORS_DIMENSIONS = [
    (M1_X, M1_Y),
    (M2_X, M2_Y),
    (M3_X, M3_Y),
    (M4_X, M4_Y),
]


TURN_VELOCITY = 0.78535 # 45degrees in 1s
MOVE_VELOCITY = 0.3

def limit_angle(angle:float) -> float:
    if angle > math.pi/2:
        return angle - math.pi
    if angle < -math.pi/2:
        return angle + math.pi
    return angle

class Deltable():

    def __init__(self):
        self.__last_value = 0
        self.__last_delta = 0
    
    def update_deltas(self, value:float):
        self.__last_delta = value - self.__last_value
        self.__last_value = value
        return self

    @property
    def last_delta(self) -> float:
        return self.__last_delta

    @property
    def last_value(self) -> float:
        return self.__last_value

class MotorPosition(Deltable):

    def __init__(self):
        Deltable.__init__(self)
        self.__x = 0
        self.__y = 0

    def update_coordinates(self, current_absolute_angle:float):
        self.__x = self.last_delta * math.cos(current_absolute_angle)
        self.__y = self.last_delta * math.sin(current_absolute_angle)
        return self

    @property
    def x(self) -> float:
        return self.__x

    @property
    def y(self) -> float:
        return self.__y

class OdomHandler():

    MOTOR_NUM = 4
    MIN_ANGLE_DIFF = 1 / 1000000

    def __init__(self, turning_point_publisher:rospy.Publisher):
        self._current_pose = Pose()
        self._curent_center_point = None

        self.__motor_positions = []
        self.__servo_angles = []
        self.__motor_coordinates = []

        self.__current_X_absolute_angle = 0
        
        for mx, my in ROBOT_MOTORS_DIMENSIONS:
            self.__motor_positions.append(Point(mx, my, 0))
            self.__servo_angles.append(0.0)
            self.__motor_coordinates.append(MotorPosition())
        
        self._turning_point_publisher = turning_point_publisher
    
    def turn_around_XY(self, turn_angle:float, x:float = 0, y:float = 0) -> List[Request]:
        moves = []

        m1_y, m1_x = M1_Y - y, M1_X - x
        m2_y, m2_x = M2_Y - y, M2_X - x
        m3_y, m3_x = M3_Y - y, M3_X - x
        m4_y, m4_x = M4_Y - y, M4_X - x

        servo_move = Request()
        servo_move.motor1.angle = round(limit_angle(M1_IN_PLACE_TURN * math.atan2(-m1_y, m1_x) - self.__servo_angles[0]), 5)
        servo_move.motor2.angle = round(limit_angle(M2_IN_PLACE_TURN * math.atan2(-m2_y, m2_x) - self.__servo_angles[1]), 5)
        servo_move.motor3.angle = round(limit_angle(M3_IN_PLACE_TURN * math.atan2(-m3_y, m3_x) - self.__servo_angles[2]), 5)
        servo_move.motor4.angle = round(limit_angle(M4_IN_PLACE_TURN * math.atan2(-m4_y, m4_x) - self.__servo_angles[3]), 5)
        max_servo_angle = max([
            abs(servo_move.motor1.angle),
            abs(servo_move.motor2.angle),
            abs(servo_move.motor3.angle),
            abs(servo_move.motor4.angle)
        ])
        servo_rotation_duration = max_servo_angle / TURN_VELOCITY

        servo_move.move_duration = round(servo_rotation_duration, 3)
        moves.append(servo_move)

        m1_radius = math.sqrt(m1_x * m1_x + m1_y * m1_y)
        m2_radius = math.sqrt(m2_x * m2_x + m2_y * m2_y)
        m3_radius = math.sqrt(m3_x * m3_x + m3_y * m3_y)
        m4_radius = math.sqrt(m4_x * m4_x + m4_y * m4_y)

        mean_radius = (m1_radius + m2_radius + m3_radius + m4_radius) / 4.0

        m1_coeficient = max(m1_radius/mean_radius, MIN_TURN_COEF)
        m2_coeficient = max(m2_radius/mean_radius, MIN_TURN_COEF)
        m3_coeficient = max(m3_radius/mean_radius, MIN_TURN_COEF)
        m4_coeficient = max(m4_radius/mean_radius, MIN_TURN_COEF)

        m1_coeficient = m1_coeficient if y > M1_Y else -m1_coeficient
        m2_coeficient = -m2_coeficient if y < M2_Y else m2_coeficient
        m3_coeficient = -m3_coeficient if y < M3_Y else m3_coeficient
        m4_coeficient = m4_coeficient if y > M4_Y else -m4_coeficient

        # m1_coeficient = m1_coeficient if x > M1_X else -m1_coeficient
        # m2_coeficient = m2_coeficient if x < M2_X else -m2_coeficient
        # m3_coeficient = m3_coeficient if x > M3_X else -m3_coeficient
        # m4_coeficient = m4_coeficient if x < M4_X else -m4_coeficient

        # m1_coeficient = m1_coeficient if y > M1_Y else -m1_coeficient
        # m2_coeficient = m2_coeficient if y > M2_Y else -m2_coeficient
        # m3_coeficient = m3_coeficient if y < M3_Y else -m3_coeficient
        # m4_coeficient = m4_coeficient if y < M4_Y else -m4_coeficient

        turn_distance = turn_angle * mean_radius # (angle/2pi) * 2piR = angle * R

        turn_duration = abs(3.75 * turn_distance / MOVE_VELOCITY)

        if turn_angle > 0:
            angle_coefficient = 1.25
        else:
            angle_coefficient = -1.25


        motor_move = Request()
        motor_move.motor1.velocity = round(angle_coefficient * m1_coeficient * MOVE_VELOCITY, 3)
        motor_move.motor2.velocity = round(angle_coefficient * m2_coeficient * MOVE_VELOCITY, 3)
        motor_move.motor3.velocity = round(angle_coefficient * m3_coeficient * MOVE_VELOCITY, 3)
        motor_move.motor4.velocity = round(angle_coefficient * m4_coeficient * MOVE_VELOCITY, 3)
        motor_move.move_duration = round(turn_duration, 3)
        moves.append(motor_move)

        return moves
    
    def move_forward(self, distance:float) -> List[Request]:
        moves = []
        
        servo_move = Request()
        servo_move.motor1.angle = 0.0
        servo_move.motor2.angle = 0.0
        servo_move.motor3.angle = 0.0
        servo_move.motor4.angle = 0.0
        max_servo_angle = max([
            abs(servo_move.motor1.angle-self.__servo_angles[0]),
            abs(servo_move.motor2.angle-self.__servo_angles[1]),
            abs(servo_move.motor3.angle-self.__servo_angles[2]),
            abs(servo_move.motor4.angle-self.__servo_angles[3])
        ])

        rospy.loginfo(f"max_servo_angle={max_servo_angle}, servo_move.motor1.angle={servo_move.motor1.angle} self.__servo_angles[0]={self.__servo_angles[0]}")
        servo_rotation_duration = max_servo_angle / TURN_VELOCITY

        servo_move.move_duration = round(servo_rotation_duration, 3)
        moves.append(servo_move)
        
        
        motor_move = Request()
        motor_move.motor1.velocity = round(MOVE_VELOCITY, 3)
        motor_move.motor2.velocity = round(MOVE_VELOCITY, 3)
        motor_move.motor3.velocity = round(MOVE_VELOCITY, 3)
        motor_move.motor4.velocity = round(MOVE_VELOCITY, 3)
        motor_move.move_duration = round(distance/MOVE_VELOCITY, 3)
        moves.append(motor_move)

        return moves
            

    def handle_motor_updates(self, response:StatusResponse) -> Pose:
        motors = [
            response.motor1,
            response.motor2,
            response.motor3,
            response.motor4
        ]
        for c, (motor_status, motor_position) in enumerate(zip(motors, self.__motor_coordinates)):
            motor_position.update_deltas(motor_status.distance).update_coordinates(self.__current_X_absolute_angle)
            self.__servo_angles[c] = motor_status.angle

        center_points = []

        for i1 in range(OdomHandler.MOTOR_NUM):
            for i2 in range(OdomHandler.MOTOR_NUM):
                if i2 >= i1:
                    continue
                
                motor_status_A, motor_position_A = motors[i1], self.__motor_positions[i1]
                motor_status_B, motor_position_B = motors[i2], self.__motor_positions[i2]

                XA = motor_position_A.x
                YA = motor_position_A.y
                alfa = motor_status_A.angle
                if i1%2 == 1:
                    alfa = -alfa

                XB = motor_position_B.x
                YB = motor_position_B.y
                beta = motor_status_B.angle
                if i2%2 == 1:
                    alfa = - alfa

                # rospy.loginfo(f"M{i1}:{motor_status_A.angle} M{i2}:{motor_status_B.angle}")
                if abs(alfa - beta) < OdomHandler.MIN_ANGLE_DIFF:
                    continue

                p = Point()
                p.y = ( YA * math.tan(math.pi/2 - alfa) + YB * math.tan(math.pi/2 - beta) + XB - XA ) / (math.tan(math.pi/2 - alfa) - math.tan(math.pi/2 - beta))
                p.x = XA - (YA - p.y) * math.tan(math.pi/2 - alfa)

                center_points.append(p)

        if center_points:
            old_point = self._curent_center_point
            self._curent_center_point = Point()
            self._curent_center_point.x = sum([p.x for p in center_points])/len(center_points)
            self._curent_center_point.y = sum([p.y for p in center_points])/len(center_points)

            if not old_point or old_point.x != self._curent_center_point.x or old_point.y != self._curent_center_point.y:
                rospy.loginfo(f"Center Point: [{self._curent_center_point.x}, {self._curent_center_point.y}, {self._curent_center_point.z}]")
            
            # center_point_stamped = PointStamped()
            # center_point_stamped.header.time 
            # self._turning_point_publisher.publish(center_point)
            
        else:
            self._curent_center_point = None




        p = Pose()
        p.orientation = get_quaterion_from_rpy(0, 0, 0)
        return p
