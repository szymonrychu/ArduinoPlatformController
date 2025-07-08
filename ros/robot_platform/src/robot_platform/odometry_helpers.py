
from .models import Motor, Servo, Request, Status
from .tf_helpers import *
from .platform_statics import PlatformStatics

from typing import List, Optional

import math
from robot_platform.msg import PlatformStatus
from geometry_msgs.msg import Point

def compute_turning_point(m_a:float, ma_x:float, ma_y:float, m_b:float, mb_x:float, mb_y:float) -> Optional[Point]:
    """
    Function computes turning point for 2 wheels based on angle and X,Y position of the wheel

    Args:
        m_a (float): motor A angle
        ma_x (float): motor A X coordinate
        ma_y (float): motor A Y coordinate
        m_b (float): motor B angle
        mb_x (float): motor B X coordinate
        mb_y (float): motor B Y coordinate

    Returns:
        Optional[Point]: returns Point if the wheels are not pararell
    """
    tg90mA = math.tan(math.pi/2 - m_a) if 0.0 != m_a else 0.0
    tg90mB = math.tan(math.pi/2 - m_b) if 0.0 != m_b else 0.0
    
    if tg90mA + tg90mB == 0:
        return None

    p = Point()
    p.x = -(tg90mA * ma_x - tg90mB * mb_x - ma_y - mb_y) / (tg90mA + tg90mB)
    p.y = -(tg90mA * ma_x - ma_y)
    return p

def compute_mean_turning_point(points:List[Point]) -> Optional[Point]:
    """
    Function takes list of Points and computes returns mean 
    coordinates for them, then returns this mean values as new Point

    Args:
        points (List[Point]): List of Points to compute mean from 

    Returns:
        Optional[Point]: returns Mean Point if the list is defined and not empty
    """
    if not points or len(points) < 1:
        return None
    
    points_len = len(points)
    p = Point()
    p.x = sum([w.x for w in points])/points_len
    p.y = sum([w.y for w in points])/points_len
    return p

def check_if_points_are_close(points:List[Point], tolerance:float = PlatformStatics.MAX_DISTANCE_TOLERANCE) -> bool:
    """
    Validates if the points are reasonably close to each other.
    Tolerance can be adjusted.

    Args:
        points (List[Point]): List of points to validate
        tolerance (float, optional): tolerance when validating the distance between points. Defaults to PlatformStatics.MAX_DISTANCE_TOLERANCE.

    Returns:
        bool: _description_
    """    
    if not points:
        return False

    point_cordinate_deltas = []
    points_distance = []
    for c_a, pa in enumerate(points):
        points_distance.append(math.sqrt(pa.x**2 + pa.y**2))
        for c_b, pb in enumerate(points):
            if c_a >= c_b:
                continue
            point_cordinate_deltas.append(pa.x - pb.x)
            point_cordinate_deltas.append(pa.y - pb.y)
    
    if not point_cordinate_deltas:
        rospy.logwarn("no point_cordinate_deltas provided")
        return False
    
    max_coordinate_delta = max([abs(c) for c in point_cordinate_deltas])
    return max_coordinate_delta < tolerance

def compute_turning_radius_yaw_delta(relative_turning_point:Point, motors:List[Motor]) -> Tuple[float, float]:
    """
    Computes turning radius and increment yaw angle delta from turning point and motor distance.
    For this computation to be valid, motors have to be turned in a way, so lines created out extending their
    axises are crossing each other in the same point (relative_turning_point)

    Args:
        relative_turning_point (Point): Point where motor axises are crossing
        motors (List[Motor]): List of motors used in the computation

    Returns:
        Tuple[float, float]: tumple with turning radius and delta yaw 
    """
    mean_distance_delta = sum([m.distance for m in motors]) / len(motors)
    turning_radius = math.sqrt(relative_turning_point.x ** 2 + relative_turning_point.y ** 2)
    yaw_delta = mean_distance_delta / turning_radius
    return (turning_radius, yaw_delta)

def compute_relative_turning_point(servos:List[Servo], turning_radius_scalling_factor:float = 0.5) -> Optional[Point]: 
    """
    Takes list of ServoStatus objects and tries to find relative turning point out of the lists

    Args:
        servos (List[ServoStatus]): List of Servo objects
        turning_radius_scalling_factor (float, optional): Scalling factor for the tolerance. Defaults to 0.5.

    Returns:
        Optional[Point]: If possible consist of a point, where the wheel axises are crossing
    """ 
    partial_turning_points = []
    for c_a, motor_a in enumerate(servos):
        for c_b, motor_b in enumerate(servos):
            if c_a >= c_b:
                continue
            (XA, YA), (XB, YB) = PlatformStatics.ROBOT_MOTORS_DIMENSIONS[c_a], PlatformStatics.ROBOT_MOTORS_DIMENSIONS[c_b]
            turning_point = compute_turning_point(motor_a.angle, XA, YA, motor_b.angle, XB, YB)
            if turning_point:
                partial_turning_points.append(turning_point)
        
    turning_point = compute_mean_turning_point(partial_turning_points)

    if turning_point:
        abs_turning_radius_scalled = turning_radius_scalling_factor * math.sqrt(turning_point.x**2 + turning_point.y**2)
        tolerance_scalled = max(abs_turning_radius_scalled, 1.0)
        if check_if_points_are_close(partial_turning_points, tolerance_scalled):
            return turning_point
    return None

def check_if_wheels_are_pararell(servos:List[Servo]) -> bool:
    """
    Checks if wheels are pararell and it's possible to move forward without steering

    Args:
        servos (List[Servo]): List of Servo objects

    Returns:
        bool: returns true if wheels are pararell enough
    """
    max_angle = max([s.angle for s in servos])
    min_angle = min([s.angle for s in servos])
    return abs(max_angle - min_angle) < 0.00000000001

def compute_target_servo_angles(turning_point:Optional[Point]=None) -> Tuple[List[float], List[bool]]:
    """
    Function takes turning point and computes list of angles that each servo of the platform should reach.
    If no turning point is provided, angles are set to 0.

    Args:
        turning_point (Optional[Point], optional): Point where lines drawed from motor axises will cross. Defaults to None.

    Returns:
       Tuple[List[float], List[bool]]: List of angles and list of bools, 1st contains angles for servos to reach, 2nd contains info if angle was reversed
    """    
    if not turning_point:
        return [0.0] * PlatformStatics.MOTOR_NUM, [False] * PlatformStatics.MOTOR_NUM
    
    relative_turning_points = []
    for (m_x, m_y) in PlatformStatics.ROBOT_MOTORS_DIMENSIONS:
        relative_turning_points.append((m_x - turning_point.x, m_y - turning_point.y))
    
    target_angles = []
    reversed_angles = []
    for (tp_x, tp_y) in relative_turning_points:
        target_angle, reversed = limit_angle(math.atan2(tp_x, tp_y))
        target_angles.append(target_angle)
        reversed_angles.append(reversed)
    
    return target_angles, reversed_angles

def compute_delta_servo_angles(target_angles:List[float], servos:List[Servo]) -> List[float]:
    """
    Computes delta between servo angles reported by platform and target angles provided

    Args:
        target_angles (List[float]): angles for platform servos to reach
        servos (List[ServoStatus]): List of Servo objects

    Returns:
        List[float]: List of delta angles
    """
    delta_servo_angles = []
    for target_angle, servo in zip(target_angles, servos):
        delta_servo_angles.append(target_angle - servo.angle)
    return delta_servo_angles

def limit_delta_servo_velocity_angles(delta_servo_angles:List[float], duration:float, turn_velocity:float = PlatformStatics.TURN_VELOCITY) -> List[float]:
    """
    Limit angle deltas based on the duration provided.

    Args:
        delta_servo_angles (List[float]): List of delta angles
        duration (float): duration around which the deltas should be scalled
        turn_velocity (float, optional): turn_velocity to limit the turn angles around. Defaults to PlatformStatics.TURN_VELOCITY.

    Returns:
        List[float]: List of limiteddelta angles
    """
    max_abs_delta_servo_angle = max([abs(a) for a in delta_servo_angles])
    if max_abs_delta_servo_angle == 0:
        return delta_servo_angles

    max_possible_servo_angle = min([max_abs_delta_servo_angle, duration * turn_velocity])
    delta_angle_coefficient = max_possible_servo_angle / max_abs_delta_servo_angle
    return [ delta_angle_coefficient * delta_servo_angle for delta_servo_angle in delta_servo_angles]

def compute_max_turning_duration(delta_servo_angles:List[float], turn_velocity:float = PlatformStatics.TURN_VELOCITY) -> float:
    """
    Compute how long it will take to turn all wheels completely towards defined angles

    Args:
        delta_servo_angles (List[float]): List of delta angles
        turn_velocity (float, optional): turn_velocity to limit the turn angles around. Defaults to PlatformStatics.TURN_VELOCITY.

    Returns:
        float: Time in seconds to fully turn the wheels
    """
    max_abs_delta_servo_angle = max([abs(a) for a in delta_servo_angles])
    max_turning_time = max_abs_delta_servo_angle/turn_velocity
    return max_turning_time

def compute_new_angle_updates(delta_servo_angles:List[float], servos:List[Servo]) -> List[float]:
    """
    Takes delta angles and computes absolute requests for the platform for the next request

    Args:
        delta_servo_angles (List[float]): List of delta angles
        servos (List[Servo]): List of Servo objects

    Returns:
        List[float]: List of angles
    """    
    target_angles = []
    for delta_angle, servo in zip(delta_servo_angles, servos):
        target_angles.append(delta_angle + servo.angle)
    return target_angles

def create_request(duration:float, platform_status:PlatformStatus, velocity:float = PlatformStatics.MOVE_VELOCITY, turn_duration:Optional[float] = None, turning_point:Optional[Point]=None, tilt:float=0.0, pan:float=0.0, in_place:bool = False) -> Request:
    """_summary_

    Args:
        duration (float): _description_
        platform_status (PlatformStatus): _description_
        velocity (float, optional): _description_. Defaults to PlatformStatics.MOVE_VELOCITY.
        turn_duration (Optional[float], optional): _description_. Defaults to None.
        turning_point (Optional[Point], optional): _description_. Defaults to None.
        tilt (float, optional): _description_. Defaults to 0.0.
        pan (float, optional): _description_. Defaults to 0.0.

    Returns:
        List[Request]: _description_
    """
    servos = [
        Servo.from_ROS_ServoStatus(platform_status.servo1),
        Servo.from_ROS_ServoStatus(platform_status.servo2),
        Servo.from_ROS_ServoStatus(platform_status.servo3),
        Servo.from_ROS_ServoStatus(platform_status.servo4),
    ]
    motor_turn_time = turn_duration or duration
    target_servo_angles, reversed_servo_angles = compute_target_servo_angles(turning_point)
    delta_servo_angles = compute_delta_servo_angles(target_servo_angles, servos)

    current_turning_point = compute_relative_turning_point(servos)
    max_turning_duration = compute_max_turning_duration(delta_servo_angles)

    request = Request.from_ROS_PlatformStatus(platform_status)
    if current_turning_point != None or max_turning_duration < duration or not in_place:
        limited_deltas = limit_delta_servo_velocity_angles(delta_servo_angles, motor_turn_time)
        motor_servo_angle_deltas = compute_new_angle_updates(limited_deltas, servos)
        request.duration = duration
        request.servo1 = Servo(angle=round(motor_servo_angle_deltas[0], 3))
        request.servo2 = Servo(angle=round(motor_servo_angle_deltas[1], 3))
        request.servo3 = Servo(angle=round(motor_servo_angle_deltas[2], 3))
        request.servo4 = Servo(angle=round(motor_servo_angle_deltas[3], 3))
        # turning_point_under_robot = False

        # turning_point_within_platform_length = -PlatformStatics.ROBOT_LENGTH/2 < current_turning_point.y < PlatformStatics.ROBOT_LENGTH/2
        # turning_point_within_platform_width = -PlatformStatics.ROBOT_WIDTH/2 < current_turning_point.x < PlatformStatics.ROBOT_WIDTH/2
        # turning_point_under_robot = turning_point_within_platform_length and turning_point_within_platform_width
        
        # if not turning_point_under_robot:
        individual_turn_radiuses = []
        for (m_x, m_y) in PlatformStatics.ROBOT_MOTORS_DIMENSIONS:
            individual_turn_radiuses.append(math.sqrt((m_y - current_turning_point.y)**2 + (m_x + current_turning_point.x)**2))
        max_individual_turn_radius = max(individual_turn_radiuses)
        
        velocity_coefficients = []
        for itr, reversed in zip(individual_turn_radiuses, reversed_servo_angles):
            c = -1.0 if reversed else 1.0
            velocity_coefficients.append(c * itr / max_individual_turn_radius)
        
        request.motor1 = Motor(velocity = round(velocity_coefficients[0] * velocity, 3))
        request.motor2 = Motor(velocity = round(velocity_coefficients[1] * velocity, 3))
        request.motor3 = Motor(velocity = round(velocity_coefficients[2] * velocity, 3))
        request.motor4 = Motor(velocity = round(velocity_coefficients[3] * velocity, 3))

    # elif check_if_wheels_are_pararell(servos):
    #     request.servo1 = None
    #     request.servo2 = None
    #     request.servo3 = None
    #     request.servo4 = None
    #     request.motor1 = Motor(velocity = round(velocity, 3))
    #     request.motor2 = Motor(velocity = round(velocity, 3))
    #     request.motor3 = Motor(velocity = round(velocity, 3))
    #     request.motor4 = Motor(velocity = round(velocity, 3))

    else:
        request = Request.from_ROS_PlatformStatus(platform_status)
        request.duration = max_turning_duration
        request.servo1 = Servo(angle=round(delta_servo_angles[0], 3))
        request.servo2 = Servo(angle=round(delta_servo_angles[1], 3))
        request.servo3 = Servo(angle=round(delta_servo_angles[2], 3))
        request.servo4 = Servo(angle=round(delta_servo_angles[3], 3))
        request.motor1 = Motor(velocity=0)
        request.motor2 = Motor(velocity=0)
        request.motor3 = Motor(velocity=0)
        request.motor4 = Motor(velocity=0)

    return request

