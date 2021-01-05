import math
class PlatformStatics():
    WHEEL_NUM = 4
    FL_WHEEL1 = '/dev/serial/by-id/usb-Teensyduino_USB_Serial_6075600-if00'
    FR_WHEEL2 = '/dev/serial/by-id/usb-Teensyduino_USB_Serial_5952630-if00'
    BL_WHEEL3 = '/dev/serial/by-id/usb-Teensyduino_USB_Serial_5437890-if00'
    BR_WHEEL4 = '/dev/serial/by-id/usb-Teensyduino_USB_Serial_5952790-if00'
    SERIAL_BAUDRATE = 115200
    ROBOT_LENGTH = 487.3
    ROBOT_WIDTH  = 322.534
    ANGLE_OFFSETS = [
        -0.1, 0.1, 0.1, -0.1
    ]
    WHEELS = [
        FL_WHEEL1, FR_WHEEL2, BL_WHEEL3, BR_WHEEL4
    ]
    WHEELS_TRANSLATIONS_XYZ = [
        (0.2436, 0.161267, 0),
        (-0.2436, 0.161267, 0),
        (0.2436, -0.161267, 0),
        (-0.2436, -0.161267, 0),
    ]
    TURNING_LIMIT = math.radians(60)
    SPEED_LIMIT = 10
    WHEEL_RADIUS = 200


    @staticmethod
    def wheel0_RPY(R, P, Y):
        if Y < 0:
            return R, P, -math.pi/2 + Y - PlatformStatics.ANGLE_OFFSETS[0]
        elif Y > 0:
            return R, P, -(-math.pi/2 + -Y - PlatformStatics.ANGLE_OFFSETS[0])
        else:
            return -math.pi/2 - PlatformStatics.ANGLE_OFFSETS[0]
    @staticmethod
    def wheel1_RPY(R, P, Y):
        # return R, P, -math.pi/2 + Y - PlatformStatics.ANGLE_OFFSETS[1]
        if Y < 0:
            return R, P, -math.pi/2 + Y - PlatformStatics.ANGLE_OFFSETS[1]
        elif Y > 0:
            return R, P, -(-math.pi/2 + -Y - PlatformStatics.ANGLE_OFFSETS[1])
        else:
            return -math.pi/2 - PlatformStatics.ANGLE_OFFSETS[1]
    @staticmethod
    def wheel2_RPY(R, P, Y):
        # return R, P, -math.pi/2 + Y - PlatformStatics.ANGLE_OFFSETS[2]
        if Y < 0:
            return R, P, -math.pi/2 + Y - PlatformStatics.ANGLE_OFFSETS[2]
        elif Y > 0:
            return R, P, -(-math.pi/2 + -Y - PlatformStatics.ANGLE_OFFSETS[2])
        else:
            return -math.pi/2 - PlatformStatics.ANGLE_OFFSETS[2]
    @staticmethod
    def wheel3_RPY(R, P, Y):
        # return R, P, -math.pi/2 + Y - PlatformStatics.ANGLE_OFFSETS[3]
        if Y < 0:
            return R, P, -math.pi/2 + Y - PlatformStatics.ANGLE_OFFSETS[3]
        elif Y > 0:
            return R, P, -(-math.pi/2 + -Y - PlatformStatics.ANGLE_OFFSETS[3])
        else:
            return -math.pi/2 - PlatformStatics.ANGLE_OFFSETS[3]
    


    WHEEL_RPY_CONVERTER = [
        wheel0_RPY.__func__,
        wheel1_RPY.__func__,
        wheel2_RPY.__func__,
        wheel3_RPY.__func__
    ]