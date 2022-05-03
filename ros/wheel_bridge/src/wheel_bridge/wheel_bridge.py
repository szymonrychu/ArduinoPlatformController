from .serial_helper import SerialWrapper
import rospy
from geometry_msgs.msg import Vector3, TransformStamped, Pose2D
import tf
import tf_conversions
import tf2_ros
import signal
import math
import time
from dataclasses import dataclass

import os

STATE_FRESH           = 'fresh'
STATE_RESET_REQUESTED = 'reset_requested'
STATE_ACCEPTING_CMDS  = 'accepting_cmds'
STATE_BUSY            = 'busy'

FRONT_WHEEL_IDS = [1, 2]
BACK_WHEEL_IDS  = [2, 3]

stateID2str = {
    0: STATE_FRESH,
    1: STATE_RESET_REQUESTED,
    2: STATE_ACCEPTING_CMDS,
    4: STATE_BUSY
}

_env2log_name = 'ROS_LOG_LEVEL'
_env2log = {
    'DEBUG': rospy.DEBUG,
    'INFO':  rospy.INFO,
    'WARN':  rospy.WARN,
    'ERROR': rospy.ERROR,
    'FATAL': rospy.FATAL
}
def env2log():
    try:
        return _env2log[os.getenv(_env2log_name, 'INFO')]
    except Exception:
        return rospy.INFO

def parse_message(raw_message, wheel_id):
    # 0000000724:INF:2 0.0006:-0.0006:0.0000:0.0250 -1 0.0142:-0.0142:0.0000:0.0250 -18 1.0050
    try:
        id_state, distance_data, distance_power, angle_data, angle_power, time_delta = raw_message.split(' ')
        # 0000000724:INF:2
        message_id, level, state = id_state.split(':')
        # 0.0006:-0.0006:0.0000:0.0250
        distance, distance_error, velocity_distance_error, velocity_distance_steering = distance_data.split(':')
        # 0.0142:-0.0142:0.0000:0.0250
        angle, angle_error, velocity_angle_error, velocity_angle_steering = angle_data.split(':')
        direction = 0
        if wheel_id in FRONT_WHEEL_IDS:
            direction = 1
        elif wheel_id in BACK_WHEEL_IDS:
            direction = -1
        return {
            'id': int(message_id),
            'level': level,
            'state': stateID2str[int(state)],
            'time_delta': float(time_delta)/1000.0,
            'distance': {
                'current': direction * float(distance),
                'error': direction *  float(distance_error),
                'velocity_error': float(velocity_distance_error),
                'velocity_steering': float(velocity_distance_steering),
                'power': float(distance_power)
            },
            'angle': {
                'current': float(angle),
                'error': float(angle_error),
                'velocity_error': float(velocity_angle_error),
                'velocity_steering': float(velocity_angle_steering),
                'power': float(angle_power)
            }
        }
    except ValueError:
        return {}


class Wheel(SerialWrapper):

    def _move_command(self, distance, angle, time):
        return "G10 {} {} {}".format(distance, angle, time)
        # G11 0 0 1

    def __init__(self, wheel_id, serial_dev, baudrate, input_topic, output_topic, tf2_base_link, tf2_output):
        SerialWrapper.__init__(self, serial_dev, baudrate=baudrate)
        self.__wheel_id = wheel_id
        self.__running = False
        self.__last_distance = 0.0
        self.__distance_set = False
        self._tf_broadcaster = tf2_ros.TransformBroadcaster()
        self._tf2_base_link = tf2_base_link
        self._tf2_output = tf2_output
        rospy.Subscriber(input_topic, Vector3, self._topic_callback)
        self._output_pub = rospy.Publisher(output_topic, Pose2D, queue_size=10)
        self._x, self._y = 0.0, 0.0
        self.__last_error_time = 0

    def _topic_callback(self, data):
        distance = data.x
        angle = data.y
        time = data.z
        cmd = self._move_command(distance, angle, time)
        self.write_data(cmd)
        rospy.loginfo(f"Wrote '{cmd}' to {self._fpath}")
    
    def stop(self, *args, **kwargs):
        self.__running = False

    def _parse(self, data):
        rospy.logdebug(f"Received '{data}' from {self._fpath}")
        d = parse_message(data, self.__wheel_id)
        if d:
            current_distance = d['distance']['current']
            current_angle = d['angle']['current']
            if not self.__distance_set:
                self.__last_distance = current_distance
                self.__distance_set = True
                return
            distance_delta = current_distance - self.__last_distance
            dx = distance_delta * math.cos(current_angle)
            dy = distance_delta * math.sin(current_angle)
            self._x += dx
            self._y += dy

            # publish tranform with angles only, so the other node can compute mean position of the platform
            # based on all wheels positions
            self._tf_broadcaster.sendTransform(self._xyzRPY2TransformStamped(0, 0, 0, 0, 0, current_angle))
            # publish position of a wheel including translation and angle, so the mena position can be computed
            distance_angle = Pose2D()
            distance_angle.x = distance_delta
            distance_angle.y = current_angle
            self._output_pub.publish(distance_angle)
            self.__last_distance = current_distance
            self.__last_error_time = 0
        else:
            float_secs = time.time()
            if float_secs - self.__last_error_time > 10:
                self.__last_error_time = float_secs
                rospy.logwarn(f"Couldn't parse data '{data}'")


    def _xyzRPY2TransformStamped(self, x, y, z, R, P, Y):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self._tf2_base_link
        t.child_frame_id = self._tf2_output
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        q = tf_conversions.transformations.quaternion_from_euler(R, P, Y)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        return t

    def process(self):
        self.__running = True
        while self.__running:
            raw_data = self.read_data()
            if raw_data is not None:
                rospy.logdebug(f"received raw: {raw_data}")
                self._parse(raw_data)

def main():
    rospy.init_node('wheel_bridge', log_level=env2log())

    serial_dev = rospy.get_param("~serial_dev")
    wheel_id = rospy.get_param("~id")
    baudrate = rospy.get_param("~baudrate")
    input_topic = rospy.get_param("~input_topic")
    output_topic = rospy.get_param("~output_topic")
    tf2_base_link = rospy.get_param("~tf2_base_link")
    tf2_output = rospy.get_param("~tf2_output")

    wheel = Wheel(wheel_id, serial_dev, baudrate, input_topic, output_topic, tf2_base_link, tf2_output)

    signal.signal(signal.SIGINT, wheel.stop)
    signal.signal(signal.SIGTERM, wheel.stop)
    Wheel(wheel_id, serial_dev, baudrate, input_topic, output_topic, tf2_base_link, tf2_output).process()