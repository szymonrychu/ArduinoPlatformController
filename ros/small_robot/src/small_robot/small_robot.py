import rospy
import time
import math
import signal

from geometry_msgs.msg import PoseStamped, Pose, TransformStamped, Point
from sensor_msgs.msg import BatteryState
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Bool, String
import tf_conversions
import json

# {
#   "micros": 1357704862,
#   "status": "ready",
#   "queue_l": 0,
#   "int_temp": 39,
#   "battery": {
#     "voltage": 11.75425625
#   },
#   "imu": {
#     "quaternion": {
#       "w": 0.998046875,
#       "x": 0.059387207,
#       "y": 0.018920898,
#       "z": 0.00378418
#     },
#     "gyroscope": {
#       "x": -0.001090831,
#       "y": 0.003272492,
#       "z": -0.001090831
#     },
#     "accelerometer": {
#       "x": -0.001090831,
#       "y": 0.003272492,
#       "z": -0.001090831
#     }
#   },
#   "gps": {
#     "fix_quality": 0,
#     "satellites": 0,
#     "dec_latitude": 0,
#     "dec_longitude": 0,
#     "speed": 0,
#     "angle": 0,
#     "altitude": 0
#   },
#   "motor1": {
#     "ready": true,
#     "servo": 0,
#     "distance": -0.00133935,
#     "distance_error": 0.00133935,
#     "distance_steering": 1,
#     "velocity": 0,
#     "velocity_error": 0.00133935,
#     "velocity_steering": 0,
#     "steering": 0
#   },
#   "motor2": {
#     "ready": true,
#     "servo": 0,
#     "distance": 0,
#     "distance_error": 0,
#     "distance_steering": 1,
#     "velocity": 0,
#     "velocity_error": 0,
#     "velocity_steering": 0,
#     "steering": 0
#   },
#   "motor3": {
#     "ready": true,
#     "servo": 0,
#     "distance": 0,
#     "distance_error": 0,
#     "distance_steering": 1,
#     "velocity": 0,
#     "velocity_error": 0,
#     "velocity_steering": 0,
#     "steering": 0
#   },
#   "motor4": {
#     "ready": true,
#     "servo": 0,
#     "distance": -0.000191336,
#     "distance_error": 0.000191336,
#     "distance_steering": 1,
#     "velocity": 0,
#     "velocity_error": 0.000191336,
#     "velocity_steering": 0,
#     "steering": 0
#   }
# }

from .lib import env2log, ROBOT_WIDTH_M, Rate, SupressedLog, RobotQuaternion, SerialWrapper, RobotMotor

    
class RobotPlatform():

    def __init__(self):
        rospy.init_node('robot', log_level=env2log())

        self.__serial_dev = rospy.get_param('~serial_dev')
        self.__serial_baudrate = rospy.get_param('~serial_baudrate')

        self.__state_ready_publisher = rospy.Publisher(rospy.get_param("~state_ready_topic"), Bool, queue_size=10)
        self.__battery_state_publisher = rospy.Publisher(rospy.get_param("~battery_state_topic"), BatteryState, queue_size=10)


        rospy.Subscriber(rospy.get_param('~raw_input_topic'), String, self.__raw_string_cb)

        self.__serial = SerialWrapper(self.__serial_dev, self.__serial_baudrate)
        rospy.Timer(rospy.Duration(0.001), self.parse_serial)
        self._motor1 = RobotMotor()
        self._motor2 = RobotMotor()
        self._motor3 = RobotMotor()
        self._motor4 = RobotMotor()

    def __raw_string_cb(self, data):
        self.__serial.write_data(data.data)

    def parse_serial(self, *args, **kwargs):
        raw_data = self.__serial.read_data()
        if not raw_data:
            return
        data = json.loads(raw_data)
        try:
        
            rospy.loginfo(raw_data)
            state = Bool()
            state.data = data['status']
            self.__state_ready_publisher.publish(state)
            battery = BatteryState()
            battery.voltage = data['battery']['voltage']
            battery.present = True
            battery.power_supply_technology = 2
            battery.power_supply_status = 2 # 1=charging 2=discharging 3=not_charging 4=full
            self.__battery_state_publisher.publish(battery)

            self._motor1.refresh(data['motor1'])
            self._motor2.refresh(data['motor2'])
            self._motor3.refresh(data['motor3'])
            self._motor4.refresh(data['motor4'])
        except KeyError:
            rospy.logwarn(raw_data)
        

    def start(self):
        rospy.spin()

    def stop(self, *args, **kwargs):
        rospy.signal_shutdown()

def main():
    robot = RobotPlatform()
    signal.signal(signal.SIGINT, robot.stop)
    signal.signal(signal.SIGTERM, robot.stop)
    robot.start()