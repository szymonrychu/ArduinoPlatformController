#!/usr/bin/env python3

import rospy
import signal

import numpy as np
import tf2_ros

from std_msgs.msg import String
from geometry_msgs.msg import Pose
from sensor_msgs.msg import BatteryState, NavSatFix, NavSatStatus, Imu

from uuid import UUID
from typing import Optional

from .log_utils import env2log
from .serial_utils import SerialWrapper
from .message_utils import parse_response, Response, MoveStatus, GPSStatus, BatteryStatus, IMUStatus, MoveRequest
from .tf_helpers import difference_between_Poses

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
        self._current_pose = Pose()
        self._currently_processed_move_uuid = None
        self._move_registry = {}

        raw_input_topic = rospy.get_param('~raw_input_topic')
        rospy.Subscriber(raw_input_topic, String, self._write_raw_data)

        goal_input_topic = rospy.get_param('~goal_move_input_topic')
        rospy.Subscriber(goal_input_topic, Pose, self.__handle_goal_pose_input_data)

        raw_output_topic = rospy.get_param('~raw_output_topic')
        self._raw_log_publisher = rospy.Publisher(raw_output_topic, String)

        battery_state_output_topic = rospy.get_param('~battery_state_output_topic')
        self._battery_state_publisher = rospy.Publisher(battery_state_output_topic, BatteryState)

        gps_state_output_topic = rospy.get_param('~gps_state_output_topic')
        self._gps_state_publisher = rospy.Publisher(gps_state_output_topic, NavSatFix)

        imu_state_output_topic = rospy.get_param('~imu_state_output_topic')
        self._imu_state_publisher = rospy.Publisher(imu_state_output_topic, Imu)

        self._tf_broadcaster = tf2_ros.TransformBroadcaster()

    def _write_raw_data(self, ros_data):
        raw_string = ros_data.data
        self.write_data(raw_string)

    def request_move(self, m:MoveRequest):
        self._move_registry[m.uuid] = m
        self._write_raw_data(m.model_dump_json())

    def __drop_move_from_registry_by_uuid(self, uuid:UUID):
        try:
            del self._move_registry[uuid]
        except KeyError as _key_error:
            rospy.logerr(f"Missing {self._currently_processed_move.uuid} move in requested moves!")

    def __get_currently_processed_move(self) -> Optional[MoveRequest]:
        if not self._currently_processed_move_uuid:
            return None
        return self._move_registry.get(self._currently_processed_move_uuid, None)

    def handle_move_status(self, status:MoveStatus):
        if not status:
            if self._currently_processed_move_uuid:
                self.__drop_move_from_registry_by_uuid(status.uuid)
            self._currently_processed_move_uuid = None
            return
        
        if self._currently_processed_move_uuid != status.uuid: # new move detected
            self.__drop_move_from_registry_by_uuid(self._currently_processed_move_uuid)

        self._currently_processed_move_uuid = status.uuid
        
        try:
            self._move_registry[status.uuid].progress = status.progress
            self._move_registry[status.uuid].part = status.part
            self._move_registry[status.uuid].max_parts = status.max_parts
        except KeyError as _key_error:
            rospy.logerr(f"Missing {self._currently_processed_move.uuid} move in requested moves!")

    def handle_gps_status(self, status:GPSStatus):
        self._gps_state_publisher.publish(status.parse_ROS_GPS('/robot'))

    def handle_battery_status(self, status:BatteryStatus):
        self._battery_state_publisher.publish(status.parse_ROS_Battery('/robot'))

    def handle_imu_status(self, status:IMUStatus):
        self._imu_state_publisher.publish(status.parse_ROS_IMU('/robot'))
        self._tf_broadcaster.sendTransform(status.parse_ROS_TF('/map', '/robot'))

    def __handle_goal_pose_input_data(self, goal_pose:Pose):
        pose_difference = difference_between_Poses(self._current_pose, goal_pose)

    def handler_error_platform_output(self, response:Response):
        rospy.logerr(response)
    
    def handler_success_platform_output(self, response:Response):
        rospy.loginfo(response)
    
    def __handler_status_platform_output(self, response:Response):
        rospy.logdebug(f'Raw platform output:\n{response.model_dump_json()}')

        self.handle_battery_status(response.battery)
        self.handle_imu_status(response.imu)

        if response.move_progress:
            self.handle_move_status(response.move_status)
        else:
            self.handle_move_status(None)
        self.handle_gps_status(response.gps)

    def parse_serial(self, raw_data:String):
        response = parse_response(raw_data)
        if not response:
            return

        raw_string = String()
        raw_string.data = raw_data
        self._raw_log_publisher.publish(raw_string)
        
        if response.message_type == 'ERROR':
            self.handler_error_platform_output(response)
        elif response.message_type == 'SUCCESS':
            self.handler_success_platform_output(response)
        elif response.message_type == 'STATUS':
            self.__handler_status_platform_output(response)
        else:
            rospy.logerr(f'Unknown message type "{response.message_type}"')


def main():
    platform = RobotPlatformRawSerialROSNode()
    signal.signal(signal.SIGINT, platform.stop)
    signal.signal(signal.SIGTERM, platform.stop)
    platform.start()


