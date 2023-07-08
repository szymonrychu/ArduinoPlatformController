import rospy
import time
import math
import signal

from geometry_msgs.msg import PoseStamped, Pose, TransformStamped, Point
from sensor_msgs.msg import BatteryState
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Bool
import tf_conversions
import json


from .lib import env2log, ROBOT_WIDTH_M, Rate, SupressedLog, RobotQuaternion, SerialWrapper

class RobotPlatform():

    def __init__(self):
        rospy.init_node('robot', log_level=env2log())

        self.__serial_dev = rospy.get_param('~serial_dev')
        self.__serial_baudrate = rospy.get_param('~serial_baudrate')

        self.__state_ready_publisher = rospy.Publisher(rospy.get_param("~state_ready_topic"), Bool, queue_size=10)
        self.__battery_state_publisher = rospy.Publisher(rospy.get_param("~battery_state_topic"), Bool, queue_size=10)

        self.__serial = SerialWrapper(self.__serial_dev, self.__serial_baudrate)
        rospy.Timer(rospy.Duration(0.001), self.parse_serial)

    def parse_serial(self, *args, **kwargs):
        raw_data = self.__serial.read_data()
        if not raw_data:
            return
        data = json.loads(raw_data)

        
        
        state = Bool()
        state.data = data['status']
        self.__state_ready_publisher.publish(state)
        battery = BatteryState()
        battery.voltage = data['battery']['voltage']
        battery.present = True
        battery.power_supply_technology = 2
        battery.power_supply_status = 2 # 1=charging 2=discharging 3=not_charging 4=full
        self.__battery_state_publisher.publish(battery)

        rospy.loginfo(raw_data)

    def start(self):
        rospy.spin()

    def stop(self):
        rospy.signal_shutdown()

def main():
    robot = RobotPlatform()
    signal.signal(signal.SIGINT, robot.stop)
    signal.signal(signal.SIGTERM, robot.stop)
    robot.start()