import math
class PlatformStatics():
    WHEEL_NUM = 4
    FL_WHEEL1 = '/dev/serial/by-id/usb-Teensyduino_USB_Serial_6075600-if00'
    FR_WHEEL2 = '/dev/serial/by-id/usb-Teensyduino_USB_Serial_5952630-if00'
    BL_WHEEL3 = '/dev/serial/by-id/usb-Teensyduino_USB_Serial_5437890-if00'
    BR_WHEEL4 = '/dev/serial/by-id/usb-Teensyduino_USB_Serial_5952790-if00'
    SERIAL_BAUDRATE = 115200
    ROBOT_LENGTH = 474.0
    ROBOT_WIDTH  = 256.0
    ANGLE_OFFSETS = [
        0, 0, 0, 0
    ]
    WHEELS = [
        FL_WHEEL1, FR_WHEEL2, BL_WHEEL3, BR_WHEEL4
    ]
    WHEELS_TRANSLATIONS_XYZ = [
        (ROBOT_LENGTH/200.0, ROBOT_WIDTH/200.0, 0),
        (-ROBOT_LENGTH/200.0, ROBOT_WIDTH/200.0, 0),
        (ROBOT_LENGTH/200.0, -ROBOT_WIDTH/200.0, 0),
        (-ROBOT_LENGTH/200.0, -ROBOT_WIDTH/200.0, 0),
    ]
    TURNING_LIMIT = math.radians(60)
    SPEED_LIMIT = 10
    WHEEL_RADIUS = 200


    @staticmethod
    def wheel0_RPY(R, P, Y):
        return R, P, Y - PlatformStatics.ANGLE_OFFSETS[0]
    @staticmethod
    def wheel1_RPY(R, P, Y):
        return R, P, Y - PlatformStatics.ANGLE_OFFSETS[1]
    @staticmethod
    def wheel2_RPY(R, P, Y):
        return R, P, Y - PlatformStatics.ANGLE_OFFSETS[2]
    @staticmethod
    def wheel3_RPY(R, P, Y):
        return R, P, Y - PlatformStatics.ANGLE_OFFSETS[3]
    


    WHEEL_RPY_CONVERTER = [
        wheel0_RPY.__func__,
        wheel1_RPY.__func__,
        wheel2_RPY.__func__,
        wheel3_RPY.__func__
    ]