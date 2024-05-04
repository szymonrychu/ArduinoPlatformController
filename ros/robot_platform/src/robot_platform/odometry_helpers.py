
from .message_utils import MotorStatus
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
    M1_Y = (ROBOT_WIDTH/2) 
    M1_X = -(ROBOT_LENGTH/2)

    # RIGHT BACK
    M2_IN_PLACE_TURN = -1.0
    M2_Y = -(ROBOT_WIDTH/2) 
    M2_X = -(ROBOT_LENGTH/2)

    # RIGHT FRONT
    M3_IN_PLACE_TURN = -1.0
    M3_ANGLE_COEFFICIENT = 1.0
    M3_Y = -(ROBOT_WIDTH/2) 
    M3_X = (ROBOT_LENGTH/2)

    # LEFT FRONT
    M4_IN_PLACE_TURN = -1.0
    M4_Y = (ROBOT_WIDTH/2) 
    M4_X = (ROBOT_LENGTH/2)

    ROBOT_MOTORS_DIMENSIONS = [
        (M1_X, M1_Y),
        (M2_X, M2_Y),
        (M3_X, M3_Y),
        (M4_X, M4_Y),
    ]


    TURN_VELOCITY = 1.04719755 # 60degrees in 1s /
    MOVE_VELOCITY = 1.5
    MIN_ANGLE_DIFF = 0.01
    MAX_DISTANCE_TOLERANCE = 0.025
    REQUEST_DURATION_COEFFICIENT = 1.5 # how much additional time to count into a move, so we get overlapped requests

    WHEEL_RADIUS = 0.13

def compute_turning_point(m_a:float, ma_x:float, ma_y:float, m_b:float, mb_x:float, mb_y:float) -> Optional[Point]:
    if abs(m_a - m_b) < PlatformStatics.MIN_ANGLE_DIFF:
        return None

    tg90mA = math.tan(math.pi/2 - m_a) if 0.0 != m_a else 0.0
    tg90mB = math.tan(math.pi/2 - m_b) if 0.0 != m_b else 0.0
    
    if tg90mA + tg90mB == 0:
        return None

    p = Point()
    # p.x = -(tg90mA * ma_y - tg90mB * mb_y - ma_x - mb_x) / (tg90mA + tg90mB)
    # p.x = - (tg90mA * ma_y - ma_x)
    p.y = -(tg90mA * ma_x - ma_y)
    return p

def compute_mean_turning_point(points:List[Point]) -> Optional[Point]:
    if not points:
        return None
    
    points_len = len(points)
    p = Point()
    p.x = sum([w.x for w in points])/points_len
    p.y = sum([w.y for w in points])/points_len
    return p

def check_if_points_are_close(points:List[Point]) -> bool:
    if not points:
        return False

    point_cordinate_deltas = []
    for c_a, pa in enumerate(points):
        for c_b, pb in enumerate(points):
            if c_a >= c_b:
                continue
            point_cordinate_deltas.append(pa.x - pb.x)
            point_cordinate_deltas.append(pa.y - pb.y)
    max_coordinate_delta = max([abs(c) for c in point_cordinate_deltas]) if point_cordinate_deltas else PlatformStatics.MAX_DISTANCE_TOLERANCE
    return max_coordinate_delta < PlatformStatics.MAX_DISTANCE_TOLERANCE


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

def compute_turning_radius_yaw_delta(relative_turning_point:Point, motors:List[Motor]) -> Tuple[float]:
    motors_num = len(motors)
    mean_distance_delta = sum([m.distance for m in motors]) / motors_num
    turning_radius = math.sqrt(relative_turning_point.x ** 2 + relative_turning_point.y ** 2)
    yaw_delta = mean_distance_delta / turning_radius
    return (turning_radius, yaw_delta)

def compute_relative_turning_point(motors:List[MotorStatus]) -> Optional[Point]:
    partial_turning_points = []
    for c_a, motor_a in enumerate(motors):
        for c_b, motor_b in enumerate(motors):
            if c_a >= c_b:
                continue
            (XA, YA), (XB, YB) = PlatformStatics.ROBOT_MOTORS_DIMENSIONS[c_a], PlatformStatics.ROBOT_MOTORS_DIMENSIONS[c_b]
            turning_point = compute_turning_point(motor_a.angle, XA, YA, motor_b.angle, XB, YB)
            if turning_point:
                partial_turning_points.append(turning_point)
    
    turning_point = None
    if check_if_points_are_close(partial_turning_points):
        turning_point = compute_mean_turning_point(partial_turning_points)
    return turning_point


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
        target_angles.append(limit_angle(math.atan2(tp_x, tp_y)))
    
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

def motor_request_to_status(request:MoveRequest) -> List[MotorStatus]:
    motors = []
    for motor_request in [request.motor1, request.motor2, request.motor3, request.motor4]:
        motor = MotorStatus()
        motor.velocity = motor_request.velocity
        motor.angle = motor_request.servo.angle
    return motors

def non_empty_request(request:MoveRequest):
    r = request.motor1.servo.angle_provided
    r = r or request.motor2.servo.angle_provided
    r = r or request.motor3.servo.angle_provided
    r = r or request.motor4.servo.angle_provided
    r = r or abs(request.motor1.velocity) > 0
    r = r or abs(request.motor2.velocity) > 0
    r = r or abs(request.motor3.velocity) > 0
    r = r or abs(request.motor4.velocity) > 0
    return r


def create_request(velocity:float, duration:float, platform_status:PlatformStatus, turning_point:Point=None) -> Optional[MoveRequest]:
    target_servo_angles = compute_target_servo_angles(turning_point)
    delta_servo_angles = compute_delta_servo_angles(target_servo_angles, platform_status)
    limited_deltas = limit_delta_servo_velocity_angles(delta_servo_angles, duration)
    motor_servo_angle_deltas = compute_new_angle_updates(limited_deltas, platform_status)
    request = MoveRequest()
    request.duration = PlatformStatics.REQUEST_DURATION_COEFFICIENT * duration

    request.motor1.servo.angle = motor_servo_angle_deltas[0]
    request.motor2.servo.angle = motor_servo_angle_deltas[1]
    request.motor3.servo.angle = motor_servo_angle_deltas[2]
    request.motor4.servo.angle = motor_servo_angle_deltas[3]
    request.motor1.servo.angle_provided = True
    request.motor2.servo.angle_provided = True
    request.motor3.servo.angle_provided = True
    request.motor4.servo.angle_provided = True
    
    can_move_wheels_continously = True
    velocity_coefficients = [1.0] * PlatformStatics.MOTOR_NUM
    
    if turning_point:
        can_move_wheels_continously = compute_relative_turning_point(motor_request_to_status(request)) != None
        turn_radius = math.sqrt(turning_point.x**2 + turning_point.y**2)
        if abs(turn_radius) > PlatformStatics.MAX_DISTANCE_TOLERANCE:
            velocity_coefficients = []
            for (m_x, m_y) in PlatformStatics.ROBOT_MOTORS_DIMENSIONS:
                motor_turn_radius = math.sqrt((m_x - turning_point.x)**2 + (m_y + turning_point.y)**2)

                is_within_robot_width = min(0, m_x) < turning_point.x and turning_point.x < max(0, m_x)
                c = -1.0 if is_within_robot_width else 1.0

                velocity_coefficients.append( c * motor_turn_radius / turn_radius)
                if can_move_wheels_continously and is_within_robot_width:
                    can_move_wheels_continously = False
    
    if can_move_wheels_continously or max(delta_servo_angles) < PlatformStatics.MIN_ANGLE_DIFF:
        request.motor1.velocity = PlatformStatics.MOVE_VELOCITY * velocity_coefficients[0] * velocity
        request.motor2.velocity = PlatformStatics.MOVE_VELOCITY * velocity_coefficients[1] * velocity
        request.motor3.velocity = PlatformStatics.MOVE_VELOCITY * velocity_coefficients[2] * velocity
        request.motor4.velocity = PlatformStatics.MOVE_VELOCITY * velocity_coefficients[3] * velocity

    if non_empty_request(request):
        return request
