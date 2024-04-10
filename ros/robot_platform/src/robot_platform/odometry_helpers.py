
from .tf_helpers import *

from typing import List, Optional

import math
from robot_platform.msg import PlatformStatus, MoveRequest, Motor
from geometry_msgs.msg import Point



class PlatformStatics:
    MOTOR_NUM = 4
    ROBOT_LENGTH = 0.350
    ROBOT_WIDTH = 0.248

    MIN_TURN_COEF = 0.0

    # LEFT BACK
    M1_IN_PLACE_TURN = -1.0
    M1_X = -(ROBOT_WIDTH/2) 
    M1_Y = -(ROBOT_LENGTH/2)

    # RIGHT BACK
    M2_IN_PLACE_TURN = -1.0
    M2_X = (ROBOT_WIDTH/2) 
    M2_Y = -(ROBOT_LENGTH/2)

    # RIGHT FRONT
    M3_IN_PLACE_TURN = -1.0
    M3_ANGLE_COEFFICIENT = 1.0
    M3_X = (ROBOT_WIDTH/2) 
    M3_Y = (ROBOT_LENGTH/2)

    # LEFT FRONT
    M4_IN_PLACE_TURN = -1.0
    M4_X = -(ROBOT_WIDTH/2) 
    M4_Y = (ROBOT_LENGTH/2)

    ROBOT_MOTORS_DIMENSIONS = [
        (M1_X, M1_Y),
        (M2_X, M2_Y),
        (M3_X, M3_Y),
        (M4_X, M4_Y),
    ]


    TURN_VELOCITY = 1.04719755 # 60degrees in 1s /
    MOVE_VELOCITY = 0.7
    MIN_ANGLE_DIFF = 0.01
    REQUEST_DURATION_COEFFICIENT = 1.5 # how much additional time to count into a move, so we get overlapped requests


def limit_angle(angle:float) -> float:
    if angle > math.pi/2:
        return angle - math.pi
    if angle < -math.pi/2:
        return angle + math.pi
    return angle

def _print_radians_in_degrees(angle_rad_list:List[float], round_:int = 4):
    r = []
    for a in angle_rad_list:
        a_deg = round(180.0 * a / math.pi, round_)
        r.append(f"{a_deg}")
    if len(angle_rad_list) > 1:
        return f"[{','.join(r)}]"
    else:
        return r[0]

def compute_relative_turning_point(m1_angle, m2_angle, m3_angle, m4_angle) -> Optional[Point]:
    motor_servo_angles = [
        m1_angle,
        m2_angle,
        m3_angle,
        m4_angle,
    ]

    partial_turning_points = []
    for c1 in range(PlatformStatics.MOTOR_NUM):
        for c2 in range(PlatformStatics.MOTOR_NUM):
            if c2 >= c1:
                continue
            yaw_A, (XA, YA) = motor_servo_angles[c1], PlatformStatics.ROBOT_MOTORS_DIMENSIONS[c1]
            yaw_B, (XB, YB) = motor_servo_angles[c2], PlatformStatics.ROBOT_MOTORS_DIMENSIONS[c2]

            if abs(yaw_A - yaw_B) < PlatformStatics.MIN_ANGLE_DIFF:
                continue
            rospy.loginfo(f"motor{c1} yaw={_print_radians_in_degrees([yaw_A])}, pos:[{XA},{YA}], motor{c2} yaw={_print_radians_in_degrees([yaw_B])}, pos:[{XB},{YB}]")

            p = Point()
            p.y = ( YA * math.tan(math.pi/2 - yaw_A) + YB * math.tan(math.pi/2 - yaw_B) + XB - XA ) / (math.tan(math.pi/2 - yaw_A) - math.tan(math.pi/2 - yaw_B))
            p.x = XA - (YA - p.y) * math.tan(math.pi/2 - yaw_A)
            rospy.loginfo(f"Computed relative turning point between {c1+1} and {c2+1} motors: [{p.x:.4f},{p.y:.4f}]")
            partial_turning_points.append(p)
    
    if not partial_turning_points:
        return None

    relative_turning_point = Point()
    relative_turning_point.x = sum([p.x for p in partial_turning_points])/len(partial_turning_points)
    relative_turning_point.y = sum([p.y for p in partial_turning_points])/len(partial_turning_points)
    rospy.loginfo(f"Computed turning point: [{relative_turning_point.x:.4f},{relative_turning_point.y:.4f}]")
    return relative_turning_point

def _if_between(x, a, b):
    return (a > x and x > b)

def _get_motor_list_from_platform_status(platform_status:PlatformStatus) -> List[Motor]:
    return [
        platform_status.motor1,
        platform_status.motor2,
        platform_status.motor3,
        platform_status.motor4
    ]


def compute_target_servo_angles(turning_point:Point=None) -> List[float]:
    if not turning_point:
        return [0.0] * PlatformStatics.MOTOR_NUM
    
    relative_turning_points = []
    for (m_x, m_y) in PlatformStatics.ROBOT_MOTORS_DIMENSIONS:
        relative_turning_points.append((m_x - turning_point.x, m_y - turning_point.y))
    
    target_angles = []
    for (tp_x, tp_y) in relative_turning_points:
        target_angles.append(limit_angle(math.atan2(-tp_y, tp_x)))
    
    return target_angles

def compute_delta_servo_angles(target_angles:List[float], platform_status:PlatformStatus) -> List[float]:
    delta_servo_angles = []
    for target_angle, motor_status in zip(target_angles, _get_motor_list_from_platform_status(platform_status)):
        delta_servo_angles.append(target_angle - motor_status.servo.angle)
    return delta_servo_angles

def limit_delta_servo_velocity_angles(delta_servo_angles:List[float], duration:float) -> List[float]:
    max_abs_delta_servo_angle = max([abs(a) for a in delta_servo_angles])
    if max_abs_delta_servo_angle == 0:
        return delta_servo_angles
    max_possible_servo_angle = min([max_abs_delta_servo_angle, duration * PlatformStatics.TURN_VELOCITY])
    delta_servo_coefficient = max_possible_servo_angle / max_abs_delta_servo_angle
    result = []
    for delta_servo_angle in delta_servo_angles:
        result.append(delta_servo_coefficient * delta_servo_angle)
    return result

def compute_new_angle_updates(delta_servo_angles:List[float], platform_status:PlatformStatus) -> List[float]:
    target_angles = []
    for delta_angle, motor_status in zip(delta_servo_angles, _get_motor_list_from_platform_status(platform_status)):
        target_angles.append(delta_angle + motor_status.servo.angle)
    return target_angles

def create_request(velocity:float, duration:float, motor_servo_angle_deltas:List[float] = None) -> MoveRequest:
    request = MoveRequest()
    request.duration = PlatformStatics.REQUEST_DURATION_COEFFICIENT * duration
    if motor_servo_angle_deltas:
        request.motor1.servo.angle = motor_servo_angle_deltas[0]
        request.motor2.servo.angle = motor_servo_angle_deltas[1]
        request.motor3.servo.angle = motor_servo_angle_deltas[2]
        request.motor4.servo.angle = motor_servo_angle_deltas[3]
        request.motor1.servo.angle_provided = True
        request.motor2.servo.angle_provided = True
        request.motor3.servo.angle_provided = True
        request.motor4.servo.angle_provided = True
    return request





def compute_next_request(velocity:float, autorepeat_rate:float, platform_status:PlatformStatus, turning_point:Point=None) -> MoveRequest:
    request = MoveRequest()
    request.duration = PlatformStatics.REQUEST_DURATION_COEFFICIENT/autorepeat_rate
    current_servo_angles = [
        platform_status.motor1.servo.angle,
        platform_status.motor2.servo.angle,
        platform_status.motor3.servo.angle,
        platform_status.motor4.servo.angle
    ]
    rospy.logdebug(f"Current servo angles[deg] {_print_radians_in_degrees(current_servo_angles)}")
    target_servo_angles = []
    if turning_point:
        m1_y, m1_x = PlatformStatics.M1_Y - turning_point.y, PlatformStatics.M1_X - turning_point.x
        m2_y, m2_x = PlatformStatics.M2_Y - turning_point.y, PlatformStatics.M2_X - turning_point.x
        m3_y, m3_x = PlatformStatics.M3_Y - turning_point.y, PlatformStatics.M3_X - turning_point.x
        m4_y, m4_x = PlatformStatics.M4_Y - turning_point.y, PlatformStatics.M4_X - turning_point.x

        target_servo_angles = [
            limit_angle(PlatformStatics.M1_ANGLE_COEFFICIENT * math.atan2(-m1_y, m1_x)),
            limit_angle(PlatformStatics.M2_ANGLE_COEFFICIENT * math.atan2(-m2_y, m2_x)),
            limit_angle(PlatformStatics.M3_ANGLE_COEFFICIENT * math.atan2(-m3_y, m3_x)),
            limit_angle(PlatformStatics.M4_ANGLE_COEFFICIENT * math.atan2(-m4_y, m4_x))
        ]
    else:
        target_servo_angles = [
            0.0,
            0.0,
            0.0,
            0.0
        ]
    rospy.logdebug(f"Target servo angles[deg] {_print_radians_in_degrees(target_servo_angles)}")
    
    delta_servo_angles = []
    for current_angle, target_angle in zip(current_servo_angles, target_servo_angles):
        delta_servo_angles.append(target_angle - current_angle)
    rospy.logdebug(f"Delta servo angles[deg] {_print_radians_in_degrees(delta_servo_angles)}")
    max_delta_servo_angle  = max(delta_servo_angles)

    
    increment_angles = []
    for delta_servo_angle, current_servo_angle, target_servo_angle in zip(delta_servo_angles, current_servo_angles, target_servo_angles):
        # delta_max_time_necessary = abs(delta_servo_angle / PlatformStatics.TURN_VELOCITY)
        # rospy.logdebug(f"Max time necessary to do the turn of the servos {delta_max_time_necessary}")
        # if delta_max_time_necessary > request.duration:
        #     increment_servo_angle = request.duration / delta_max_time_necessary * delta_servo_angle
        #     increment_angles.append(increment_servo_angle + current_servo_angle)
        # else:
        increment_angles.append(target_servo_angle)
    rospy.logdebug(f"Current servo angles with increment[deg] {_print_radians_in_degrees(increment_angles)}")
    
    request.motor1.servo.angle = increment_angles[0]
    request.motor2.servo.angle = increment_angles[1]
    request.motor3.servo.angle = increment_angles[2]
    request.motor4.servo.angle = increment_angles[3]
    request.motor1.servo.angle_provided = abs(increment_angles[0] - platform_status.motor1.servo.angle) > PlatformStatics.MIN_ANGLE_DIFF
    request.motor2.servo.angle_provided = abs(increment_angles[1] - platform_status.motor2.servo.angle) > PlatformStatics.MIN_ANGLE_DIFF
    request.motor3.servo.angle_provided = abs(increment_angles[2] - platform_status.motor3.servo.angle) > PlatformStatics.MIN_ANGLE_DIFF
    request.motor4.servo.angle_provided = abs(increment_angles[3] - platform_status.motor4.servo.angle) > PlatformStatics.MIN_ANGLE_DIFF

    next_turning_point = turning_point
    if next_turning_point and (abs(next_turning_point.x) > 0.001 or abs(next_turning_point.y) > 0.001):
        turn_radius = math.sqrt(next_turning_point.x**2 + next_turning_point.y**2)
        m1_radius = math.sqrt((PlatformStatics.M1_X - next_turning_point.x)**2 + (PlatformStatics.M1_Y + next_turning_point.y)**2)
        m2_radius = math.sqrt((PlatformStatics.M2_X - next_turning_point.x)**2 + (PlatformStatics.M2_Y + next_turning_point.y)**2)
        m3_radius = math.sqrt((PlatformStatics.M3_X - next_turning_point.x)**2 + (PlatformStatics.M3_Y + next_turning_point.y)**2)
        m4_radius = math.sqrt((PlatformStatics.M4_X - next_turning_point.x)**2 + (PlatformStatics.M4_Y + next_turning_point.y)**2)
        rospy.logdebug(f"platform/m1/m2/m3/m4 turning radiuses: {turn_radius:.4f}/{m1_radius:.4f}/{m2_radius:.4f}/{m3_radius:.4f}/{m4_radius:.4f}")

        m1_coeficient = -max(m1_radius/turn_radius, PlatformStatics.MIN_TURN_COEF)
        m2_coeficient = -max(m2_radius/turn_radius, PlatformStatics.MIN_TURN_COEF)
        m3_coeficient = -max(m3_radius/turn_radius, PlatformStatics.MIN_TURN_COEF)
        m4_coeficient = -max(m4_radius/turn_radius, PlatformStatics.MIN_TURN_COEF)
        
        m1_coeficient = m1_coeficient if _if_between(next_turning_point.x, 0, PlatformStatics.M1_X) else -m1_coeficient
        m2_coeficient = m2_coeficient if _if_between(next_turning_point.x, PlatformStatics.M2_X, 0) else -m2_coeficient
        m3_coeficient = m3_coeficient if _if_between(next_turning_point.x, PlatformStatics.M3_X, 0) else -m3_coeficient
        m4_coeficient = m4_coeficient if _if_between(next_turning_point.x, 0, PlatformStatics.M4_X) else -m4_coeficient

        if max_delta_servo_angle < PlatformStatics.MIN_ANGLE_DIFF:
            request.motor1.velocity = PlatformStatics.MOVE_VELOCITY * m1_coeficient * velocity
            request.motor2.velocity = PlatformStatics.MOVE_VELOCITY * m2_coeficient * velocity
            request.motor3.velocity = PlatformStatics.MOVE_VELOCITY * m3_coeficient * velocity
            request.motor4.velocity = PlatformStatics.MOVE_VELOCITY * m4_coeficient * velocity
    else:
        if max_delta_servo_angle < PlatformStatics.MIN_ANGLE_DIFF:
            request.motor1.velocity = PlatformStatics.MOVE_VELOCITY * velocity
            request.motor2.velocity = PlatformStatics.MOVE_VELOCITY * velocity
            request.motor3.velocity = PlatformStatics.MOVE_VELOCITY * velocity
            request.motor4.velocity = PlatformStatics.MOVE_VELOCITY * velocity
    
    rospy.logdebug(f"m1/m2/m3/m4 motor velocities: {request.motor1.velocity}/{request.motor2.velocity}/{request.motor3.velocity}/{request.motor4.velocity}")

    return request






# class Deltable():

#     def __init__(self):
#         self.__last_value = 0
#         self.__last_delta = 0
    
#     def update_deltas(self, value:float):
#         self.__last_delta = value - self.__last_value
#         self.__last_value = value
#         return self

#     @property
#     def last_delta(self) -> float:
#         return self.__last_delta

#     @property
#     def last_value(self) -> float:
#         return self.__last_value

# class MotorPosition(Deltable):

#     def __init__(self):
#         Deltable.__init__(self)
#         self.__x = 0
#         self.__y = 0

#     def update_coordinates(self, current_absolute_angle:float):
#         self.__x = self.last_delta * math.cos(current_absolute_angle)
#         self.__y = self.last_delta * math.sin(current_absolute_angle)
#         return self

#     @property
#     def x(self) -> float:
#         return self.__x

#     @property
#     def y(self) -> float:
#         return self.__y

# class OdomHandler():

#     MOTOR_NUM = 4
#     MIN_ANGLE_DIFF = 1 / 1000000

#     def __init__(self, turning_point_publisher:rospy.Publisher):
#         self._current_pose = Pose()
#         self._curent_center_point = None

#         self.__motor_positions = []
#         self.__servo_angles = []
#         self.__motor_coordinates = []

#         self.__current_X_absolute_angle = 0
        
#         for mx, my in ROBOT_MOTORS_DIMENSIONS:
#             self.__motor_positions.append(Point(mx, my, 0))
#             self.__servo_angles.append(0.0)
#             self.__motor_coordinates.append(MotorPosition())
        
#         self._turning_point_publisher = turning_point_publisher
    
#     def turn_around_XY(self, turn_angle:float, x:float = 0, y:float = 0) -> List[Request]:
#         moves = []

#         m1_y, m1_x = M1_Y - y, M1_X - x
#         m2_y, m2_x = M2_Y - y, M2_X - x
#         m3_y, m3_x = M3_Y - y, M3_X - x
#         m4_y, m4_x = M4_Y - y, M4_X - x

#         servo_move = Request()
#         servo_move.motor1.angle = round(limit_angle(M1_IN_PLACE_TURN * math.atan2(-m1_y, m1_x) - self.__servo_angles[0]), 5)
#         servo_move.motor2.angle = round(limit_angle(M2_IN_PLACE_TURN * math.atan2(-m2_y, m2_x) - self.__servo_angles[1]), 5)
#         servo_move.motor3.angle = round(limit_angle(M3_IN_PLACE_TURN * math.atan2(-m3_y, m3_x) - self.__servo_angles[2]), 5)
#         servo_move.motor4.angle = round(limit_angle(M4_IN_PLACE_TURN * math.atan2(-m4_y, m4_x) - self.__servo_angles[3]), 5)
#         max_servo_angle = max([
#             abs(servo_move.motor1.angle),
#             abs(servo_move.motor2.angle),
#             abs(servo_move.motor3.angle),
#             abs(servo_move.motor4.angle)
#         ])
#         servo_rotation_duration = max_servo_angle / TURN_VELOCITY

#         servo_move.move_duration = round(servo_rotation_duration, 3)
#         moves.append(servo_move)

#         m1_radius = math.sqrt(m1_x * m1_x + m1_y * m1_y)
#         m2_radius = math.sqrt(m2_x * m2_x + m2_y * m2_y)
#         m3_radius = math.sqrt(m3_x * m3_x + m3_y * m3_y)
#         m4_radius = math.sqrt(m4_x * m4_x + m4_y * m4_y)

#         mean_radius = (m1_radius + m2_radius + m3_radius + m4_radius) / 4.0

#         m1_coeficient = max(m1_radius/mean_radius, MIN_TURN_COEF)
#         m2_coeficient = max(m2_radius/mean_radius, MIN_TURN_COEF)
#         m3_coeficient = max(m3_radius/mean_radius, MIN_TURN_COEF)
#         m4_coeficient = max(m4_radius/mean_radius, MIN_TURN_COEF)

#         m1_coeficient = m1_coeficient if y > M1_Y else -m1_coeficient
#         m2_coeficient = -m2_coeficient if y < M2_Y else m2_coeficient
#         m3_coeficient = -m3_coeficient if y < M3_Y else m3_coeficient
#         m4_coeficient = m4_coeficient if y > M4_Y else -m4_coeficient

#         # m1_coeficient = m1_coeficient if x > M1_X else -m1_coeficient
#         # m2_coeficient = m2_coeficient if x < M2_X else -m2_coeficient
#         # m3_coeficient = m3_coeficient if x > M3_X else -m3_coeficient
#         # m4_coeficient = m4_coeficient if x < M4_X else -m4_coeficient

#         # m1_coeficient = m1_coeficient if y > M1_Y else -m1_coeficient
#         # m2_coeficient = m2_coeficient if y > M2_Y else -m2_coeficient
#         # m3_coeficient = m3_coeficient if y < M3_Y else -m3_coeficient
#         # m4_coeficient = m4_coeficient if y < M4_Y else -m4_coeficient

#         turn_distance = turn_angle * mean_radius # (angle/2pi) * 2piR = angle * R

#         turn_duration = abs(3.75 * turn_distance / MOVE_VELOCITY)

#         if turn_angle > 0:
#             angle_coefficient = 1.25
#         else:
#             angle_coefficient = -1.25


#         motor_move = Request()
#         motor_move.motor1.velocity = round(angle_coefficient * m1_coeficient * MOVE_VELOCITY, 3)
#         motor_move.motor2.velocity = round(angle_coefficient * m2_coeficient * MOVE_VELOCITY, 3)
#         motor_move.motor3.velocity = round(angle_coefficient * m3_coeficient * MOVE_VELOCITY, 3)
#         motor_move.motor4.velocity = round(angle_coefficient * m4_coeficient * MOVE_VELOCITY, 3)
#         motor_move.move_duration = round(turn_duration, 3)
#         moves.append(motor_move)

#         return moves
    
#     def move_forward(self, distance:float) -> List[Request]:
#         moves = []
        
#         servo_move = Request()
#         servo_move.motor1.angle = 0.0
#         servo_move.motor2.angle = 0.0
#         servo_move.motor3.angle = 0.0
#         servo_move.motor4.angle = 0.0
#         max_servo_angle = max([
#             abs(servo_move.motor1.angle-self.__servo_angles[0]),
#             abs(servo_move.motor2.angle-self.__servo_angles[1]),
#             abs(servo_move.motor3.angle-self.__servo_angles[2]),
#             abs(servo_move.motor4.angle-self.__servo_angles[3])
#         ])

#         rospy.loginfo(f"max_servo_angle={max_servo_angle}, servo_move.motor1.angle={servo_move.motor1.angle} self.__servo_angles[0]={self.__servo_angles[0]}")
#         servo_rotation_duration = max_servo_angle / TURN_VELOCITY

#         servo_move.move_duration = round(servo_rotation_duration, 3)
#         moves.append(servo_move)
        
        
#         motor_move = Request()
#         motor_move.motor1.velocity = round(MOVE_VELOCITY, 3)
#         motor_move.motor2.velocity = round(MOVE_VELOCITY, 3)
#         motor_move.motor3.velocity = round(MOVE_VELOCITY, 3)
#         motor_move.motor4.velocity = round(MOVE_VELOCITY, 3)
#         motor_move.move_duration = round(distance/MOVE_VELOCITY, 3)
#         moves.append(motor_move)

#         return moves
            

#     def handle_motor_updates(self, response:StatusResponse) -> Pose:
#         motors = [
#             response.motor1,
#             response.motor2,
#             response.motor3,
#             response.motor4
#         ]
#         for c, (motor_status, motor_position) in enumerate(zip(motors, self.__motor_coordinates)):
#             motor_position.update_deltas(motor_status.distance).update_coordinates(self.__current_X_absolute_angle)
#             self.__servo_angles[c] = motor_status.angle

#         center_points = []

#         for i1 in range(OdomHandler.MOTOR_NUM):
#             for i2 in range(OdomHandler.MOTOR_NUM):
#                 if i2 >= i1:
#                     continue
                
#                 motor_status_A, motor_position_A = motors[i1], self.__motor_positions[i1]
#                 motor_status_B, motor_position_B = motors[i2], self.__motor_positions[i2]

#                 XA = motor_position_A.x
#                 YA = motor_position_A.y
#                 alfa = motor_status_A.angle
#                 if i1%2 == 1:
#                     alfa = -alfa

#                 XB = motor_position_B.x
#                 YB = motor_position_B.y
#                 beta = motor_status_B.angle
#                 if i2%2 == 1:
#                     alfa = - alfa

#                 # rospy.loginfo(f"M{i1}:{motor_status_A.angle} M{i2}:{motor_status_B.angle}")
#                 if abs(alfa - beta) < OdomHandler.MIN_ANGLE_DIFF:
#                     continue

#                 p = Point()
#                 p.y = ( YA * math.tan(math.pi/2 - alfa) + YB * math.tan(math.pi/2 - beta) + XB - XA ) / (math.tan(math.pi/2 - alfa) - math.tan(math.pi/2 - beta))
#                 p.x = XA - (YA - p.y) * math.tan(math.pi/2 - alfa)

#                 center_points.append(p)

#         if center_points:
#             old_point = self._curent_center_point
#             self._curent_center_point = Point()
#             self._curent_center_point.x = sum([p.x for p in center_points])/len(center_points)
#             self._curent_center_point.y = sum([p.y for p in center_points])/len(center_points)

#             if not old_point or old_point.x != self._curent_center_point.x or old_point.y != self._curent_center_point.y:
#                 rospy.loginfo(f"Center Point: [{self._curent_center_point.x}, {self._curent_center_point.y}, {self._curent_center_point.z}]")
            
#             # center_point_stamped = PointStamped()
#             # center_point_stamped.header.time 
#             # self._turning_point_publisher.publish(center_point)
            
#         else:
#             self._curent_center_point = None




#         p = Pose()
#         p.orientation = get_quaterion_from_rpy(0, 0, 0)
#         return p
