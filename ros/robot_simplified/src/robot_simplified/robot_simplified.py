from multiprocessing.sharedctypes import Value
from turtle import right
from .serial_helper import SerialWrapper
import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped, Transform, Pose2D
import tf
import tf_conversions
import tf2_ros
import signal
import math
import time
from dataclasses import dataclass
import traceback

import os

ROBOT_STATE_READY         = 'ready'
STATE_MOVING_FORWARD      = 'moving_forward'
STATE_MOVING_BACKWARD     = 'moving_backward'
STATE_BUSY_MOVING_TURNING = 'moving_turning'

ROBOT_WIDTH_M = 0.23

stateID2str = {
    0: ROBOT_STATE_READY,
    1: STATE_MOVING_FORWARD,
    2: STATE_MOVING_BACKWARD,
    3: STATE_BUSY_MOVING_TURNING
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

        

class Message():
    @property
    def type(self):
        return type(self).__name__

class MoveMessage():

    def __init__(self, raw_message):
        try:
            # 0000000026:INF:0.0038:0:0:0.0038:0:0:1.0000:0
            message_id, level, l_position, l_power, l_reached, r_position, r_power, r_reached, t_delta, state = raw_message.split(':')
            self._message_id, self._level = int(message_id), level
            self._left_distance, self._left_power, self._left_reached = float(l_position)/100.0, int(l_power), True if l_reached == '0' else False
            self._right_distance, self._right_power, self._right_reached = float(r_position)/100.0, int(r_power), True if r_reached == '0' else False
            self._time_delta, self._state = float(t_delta), stateID2str[int(state)]
        except ValueError:
            tb = traceback.format_exc()
            rospy.logwarn(f"Invalid message '{raw_message}'\n{str(tb)}")

            self._message_id, self._level = None, None
            self._left_distance, self._left_power, self._left_reached = None, None, None
            self._right_distance, self._right_power, self._right_reached = None, None, None
            self._time_delta, self._state = None, None

    @property
    def id(self):
        return self._message_id
    
    @property
    def level(self):
        return self._level
    
    @property
    def left_distance(self):
        return self._left_distance
    
    @property
    def left_power(self):
        return self._left_power
    
    @property
    def left_reached(self):
        return self._left_reached
    
    @property
    def right_distance(self):
        return self._right_distance
    
    @property
    def right_power(self):
        return self._right_power
    
    @property
    def right_reached(self):
        return self._right_reached
    
    @property
    def time_deltaMS(self):
        return self._time_delta
    
    @property
    def state(self):
        return self._state

    def get_delta_distance_output_angle(self, previous_msg, Y, X, THETA):
        new_THETA = THETA
        new_X = X
        new_Y = Y
        if self.left_distance and self.left_distance and previous_msg.left_distance and previous_msg.right_distance: # and self.left_distance * self.right_distance > 0:
            delta_left = (self.left_distance - previous_msg.left_distance)
            delta_right = (self.right_distance - previous_msg.right_distance)

            delta_distance = (delta_left + delta_right) / 2

            theta = (delta_left - delta_right) / ROBOT_WIDTH_M

            if delta_distance != 0:
                distance_traveled_x = math.cos(theta) * delta_distance
                distance_traveled_y = math.sin(theta) * delta_distance

                new_X = X + (distance_traveled_x * math.cos(THETA) - distance_traveled_y * math.sin(THETA))
                new_Y = Y + (distance_traveled_x * math.sin(THETA) + distance_traveled_y * math.cos(THETA))
            
            if theta != 0:
                new_THETA = theta + THETA

        return new_Y, new_X, new_THETA
        

class RobotPlatform(SerialWrapper):

    def _move_command(self, left_distance, right_distance, time):
        return "G10 {} {} {}".format(left_distance, right_distance, time)

    def __init__(self, serial_dev, baudrate, input_topic, tf2_link_base, tf2_link_child):
        SerialWrapper.__init__(self, serial_dev, baudrate=baudrate)
        self._tf2_link_base = tf2_link_base
        self._tf2_link_child = tf2_link_child
        self._tf_broadcaster = tf2_ros.TransformBroadcaster()
        rospy.Subscriber(input_topic, PoseStamped, self._goal_callback)
        self._prev_msg = None
        
        self._x, self._y = 0.0, 0.0
        self._current_Pose2d = Pose2D()
        self._current_Pose2d.x = 0.0
        self._current_Pose2d.y = 0.0
        self.last_YAW = 0.0

        self.__running = False
        self.__wheel_state = -1
        self.__last_distance = 0.0
        
        self.__distance_set = False
        self._tf2_base_link = tf2_link_base
        self._tf2_output = tf2_link_child
        self.__last_error_time = 0

    def _goal_callback(self, data):
        rospy.loginfo(f"{data.pose.position.x} {data.pose.position.y} {data.pose.position.z}")

        dx = data.pose.position.x - self._current_Pose2d.x
        dy = data.pose.position.y - self._current_Pose2d.y

        r, p, y = tf_conversions.transformations.euler_from_quaternion([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])
        
        angle = self.last_YAW + math.atan2(dy, dx)
        distance = math.sqrt(dx*dx + dy*dy)

        rospy.loginfo(f"Rotating {angle} and moving {distance}")


        turn_distance = (angle * ROBOT_WIDTH_M /2) * 100.0
        turn_time = abs(turn_distance/2)
        turn_command = self._move_command(-turn_distance, turn_distance, turn_time)
        self.write_data(turn_command)
        rospy.loginfo(f"Turning with command '{turn_command}'")
        time.sleep( turn_time )

        move_time = 100.0 * distance / 2
        move_command = self._move_command(distance*100.0, distance*100.0, move_time)
        self.write_data(move_command)
        rospy.loginfo(f"Moving forward with command '{move_command}'")
        time.sleep( move_time )
    
    def stop(self, *args, **kwargs):
        rospy.loginfo(f"Stopping gracefully.")
        self.__running = False

    def _parse(self, data):
        rospy.logdebug(f"Received '{data}'")
        msg = MoveMessage(data)

        if self._prev_msg:
            self._current_Pose2d.y, self._current_Pose2d.x, self.last_YAW = msg.get_delta_distance_output_angle(self._prev_msg, self._current_Pose2d.y, self._current_Pose2d.x, self.last_YAW)

            # rospy.loginfo(f"Computed X/Y/yaw/state {self._current_Pose2d.x}/{self._current_Pose2d.y}/{self.last_YAW}/{msg.state}")

            platform_XY_transform = TransformStamped()
            platform_XY_transform.header.stamp = rospy.Time.now()
            platform_XY_transform.header.frame_id = 'platform_XY'
            platform_XY_transform.child_frame_id = self._tf2_link_child # self._tf2_link_child
            platform_XY_transform.transform.translation.x = -self._current_Pose2d.x
            platform_XY_transform.transform.translation.y = self._current_Pose2d.y
            platform_XY_transform.transform.translation.z = 0.0
            q = tf_conversions.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
            platform_XY_transform.transform.rotation.x = q[0]
            platform_XY_transform.transform.rotation.y = q[1]
            platform_XY_transform.transform.rotation.z = q[2]
            platform_XY_transform.transform.rotation.w = q[3]
            self._tf_broadcaster.sendTransform(platform_XY_transform)

            platform_YAW_transform = TransformStamped()
            platform_YAW_transform.header.stamp = rospy.Time.now()
            platform_YAW_transform.header.frame_id = self._tf2_link_base
            platform_YAW_transform.child_frame_id = 'platform_XY'
            q = tf_conversions.transformations.quaternion_from_euler(0.0, 0.0, self.last_YAW)
            platform_YAW_transform.transform.rotation.x = q[0]
            platform_YAW_transform.transform.rotation.y = q[1]
            platform_YAW_transform.transform.rotation.z = q[2]
            platform_YAW_transform.transform.rotation.w = q[3]
            self._tf_broadcaster.sendTransform(platform_YAW_transform)

        if msg:
            self._prev_msg = msg

    def process(self):
        self.__running = True
        while self.__running:
            raw_data = self.read_data()
            if raw_data is not None and raw_data != '':
                rospy.logdebug(f"received raw: {raw_data}")
                self._parse(raw_data)

def main():
    rospy.init_node('robot_simplified', log_level=env2log())

    serial_dev = rospy.get_param("~serial_dev")
    baudrate = rospy.get_param("~baudrate")
    input_topic = rospy.get_param("~tf2_input_topic")
    tf2_link_base = rospy.get_param("~tf2_base_link")
    tf2_link_child = rospy.get_param("~tf2_map_link")

    robot = RobotPlatform(serial_dev, baudrate, input_topic, tf2_link_base, tf2_link_child)
    signal.signal(signal.SIGINT, robot.stop)
    signal.signal(signal.SIGTERM, robot.stop)
    robot.process()