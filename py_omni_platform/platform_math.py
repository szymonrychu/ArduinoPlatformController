from .serial_helper import ThreadedSerialWrapper
from .platform_statics import PlatformStatics
import math

class PlatformMath(PlatformStatics):

    def __init__(self):
        self._angle_functions = [
            self._get_fl1_angle,
            self._get_fr2_angle,
            self._get_bl3_angle,
            self._get_br4_angle,
        ]


    def __get_closer_angle(angl_radians):
        return math.atan(PlatformMath.ROBOT_LENGTH/((PlatformMath.ROBOT_LENGTH/math.tan(angl_radians/2))-PlatformMath.ROBOT_WIDTH))

    def __get_farther_angle(angl_radians):
        return math.atan(PlatformMath.ROBOT_LENGTH/(PlatformMath.ROBOT_WIDTH-(PlatformMath.ROBOT_LENGTH/math.tan(angl_radians/2))))

    def __get_turn_radius(angl_radians):
        return PlatformMath.ROBOT_LENGTH/(2*math.cos(math.pi/2 - angl_radians))

    def _get_turn_scaler(main_angle, angl_radians):
        center_radius = __get_turn_radius(main_angle)
        wheel_radius =  __get_turn_radius(angl_radians)
        return abs(wheel_radius/center_radius)

    def _get_fl1_angle(angl_radians):
        if angl_radians < 0:
            return PlatformMath.ANGLE_OFFSETS[0] + self.__get_closer_angle(-angl_radians)
        elif angl_radians > 0:
            return PlatformMath.ANGLE_OFFSETS[0] + self.__get_closer_angle(-angl_radians)
        else:
            return PlatformMath.ANGLE_OFFSETS[0]


    def _get_fr2_angle(angl_radians):
        if angl_radians < 0:
            return PlatformMath.ANGLE_OFFSETS[1] + self.__get_farther_angle(angl_radians)
        elif angl_radians > 0:
            return PlatformMath.ANGLE_OFFSETS[1] + self.__get_farther_angle(angl_radians)
        else:
            return PlatformMath.ANGLE_OFFSETS[1]

    def _get_bl3_angle(angl_radians):
        if angl_radians < 0:
            return PlatformMath.ANGLE_OFFSETS[2] - self.__get_closer_angle(-angl_radians)
        elif angl_radians > 0:
            return PlatformMath.ANGLE_OFFSETS[2] - self.__get_closer_angle(-angl_radians)
        else:
            return PlatformMath.ANGLE_OFFSETS[2]

    def _get_br4_angle(angl_radians):
        if angl_radians < 0:
            return PlatformMath.ANGLE_OFFSETS[3] - self.__get_farther_angle(angl_radians)
        elif angl_radians > 0:
            return PlatformMath.ANGLE_OFFSETS[3] - self.__get_farther_angle(angl_radians)
        else:
            return PlatformMath.ANGLE_OFFSETS[3]

    def compute_angles_distances(self, angle, distance):
        if abs(angle) > PlatformMath.TURNING_LIMIT:
            raise ValueError(f"Angle {angle} is exceeding limit +/-{PlatformMath.TURNING_LIMIT}!")
        angles = []
        differences = []
        distances = []
        max_diff = 0
        for c in range(PlatformMath.WHEEL_NUM):
            angle = self._angle_functions[c](math.radians(a))
            difference = self._get_turn_scaler(a, angle-ANGLE_OFFSETS[c])
            angles.append(angle)
            differences.append(difference)
        max_diff = max(differences)
        for c, (angle, difference) in enumerate(zip(angles, differences)):
            normalized_difference = difference/max_diff
            wheel_distance = distance * normalized_difference
            distances.append(wheel_distance)
        result = []
        for angle, distance in zip(angles, distances):
            result.append((angle, distance,))
        return result
        

