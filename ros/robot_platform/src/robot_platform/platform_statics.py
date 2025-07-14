import math

class PlatformStatics:
    '''
    Class describing statics used in the robot
    '''
    MOTOR_NUM = 4
    ROBOT_LENGTH = 0.360
    ROBOT_WIDTH = 0.290

    MIN_TURN_COEF = 0.0

    # LEFT BACK
    M1_IN_PLACE_TURN = -1.0
    M1_X = (ROBOT_LENGTH/2)
    M1_Y = -(ROBOT_WIDTH/2)

    # RIGHT BACK
    M2_IN_PLACE_TURN = -1.0
    M2_X = (ROBOT_LENGTH/2)
    M2_Y = (ROBOT_WIDTH/2)

    # RIGHT FRONT
    M3_IN_PLACE_TURN = -1.0
    M3_ANGLE_COEFFICIENT = 1.0
    M3_X = -(ROBOT_LENGTH/2)
    M3_Y = (ROBOT_WIDTH/2)

    # LEFT FRONT
    M4_IN_PLACE_TURN = -1.0
    M4_X = -(ROBOT_LENGTH/2)
    M4_Y = -(ROBOT_WIDTH/2) 

    ROBOT_MOTORS_DIMENSIONS = [
        (M1_X, M1_Y),
        (M2_X, M2_Y),
        (M3_X, M3_Y),
        (M4_X, M4_Y),
    ]


    TURN_VELOCITY = math.pi/(0.18*3)
    MOVE_VELOCITY = 1.5
    MIN_ANGLE_DIFF = 0.01
    MAX_DISTANCE_TOLERANCE = 0.025
    DURATION_OVERLAP_STATIC=1.1
    WHEEL_RADIUS = 0.13
    SLOW_SPEED = 0.1
    MAX_SPEED = 1.0