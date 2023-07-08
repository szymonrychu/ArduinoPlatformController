import rospy
import time
import math
import signal

from geometry_msgs.msg import PoseStamped, Pose, TransformStamped, Point
from tf2_ros import TransformBroadcaster
import tf_conversions


from .lib import env2log, ROBOT_WIDTH_M, Rate, SupressedLog, RobotQuaternion, SerialWrapper

class RobotPlatform():

    def __init__(self):
        rospy.init_node('robot', log_level=env2log())

        self.__serial_dev = rospy.get_param('~serial_dev')
        self.__serial_baudrate = rospy.get_param('~serial_baudrate')

        self.__serial = SerialWrapper(self.__serial_dev, self.__serial_baudrate)
        rospy.Timer(rospy.Duration(0.001), self.parse_serial)

    def parse_serial(self, *args, **kwargs):
        raw_data = self.__serial.read_data()
        if not raw_data:
            return
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