#!/usr/bin/env python3

import rospy
import signal

from std_msgs.msg import String

from .log_utils import env2log
from .serial_utils import SerialWrapper
from .message_utils import parse_response

class ROSNode():

    def __init__(self, node_name='robot_platform'):
        rospy.init_node(node_name, log_level=env2log())

    def start(self):
        rospy.spin()

    def is_running(self):
        return not rospy.is_shutdown()

    def stop(self, reason='', *_args, **_kwargs):
        rospy.signal_shutdown(reason)


class SerialROSNode(ROSNode, SerialWrapper):

    def __init__(self):
        ROSNode.__init__(self)
        serial_dev = rospy.get_param('~serial_dev')
        serial_baudrate = rospy.get_param('~serial_baudrate')
        SerialWrapper.__init__(self, serial_dev, serial_baudrate)
    
    def start(self):
        rospy.Timer(rospy.Duration(0.001), self.__handle_serial)
        ROSNode.start(self)

    def __handle_serial(self, *_args, **_kwargs):
        raw_data = self.read_data()
        if raw_data:
            self.parse_serial(raw_data)

    def parse_serial(self, raw_data):
        pass

class RobotPlatformRawSerialROSNode(SerialROSNode):

    def __init__(self):
        SerialROSNode.__init__(self)
        raw_input_topic = rospy.get_param('~raw_input_topic')
        raw_output_topic = rospy.get_param('~raw_output_topic')
        rospy.Subscriber(raw_input_topic, String, self._write_raw_data)
        self._raw_log_publisher = rospy.Publisher(raw_output_topic, String)

    def _write_raw_data(self, ros_data):
        raw_string = ros_data.data
        self.write_data(raw_string)

    def parse_serial(self, raw_data):
        response = parse_response(raw_data)
        if not response:
            return
        
        if response.message_type == 'ERROR':
            rospy.logerr(response)
        elif response.message_type == 'SUCCESS':
            rospy.loginfo(response)
        else:
            rospy.logdebug(response)
        
        raw_string = String()
        raw_string.data = raw_data
        self._raw_log_publisher.publish(raw_string)


def main():
    platform = RobotPlatformRawSerialROSNode()
    signal.signal(signal.SIGINT, platform.stop)
    signal.signal(signal.SIGTERM, platform.stop)
    platform.start()


