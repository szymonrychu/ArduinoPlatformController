from multiprocessing.sharedctypes import Value
from .serial_helper import SerialWrapper, Rate, TransformBroadcaster, time_ms
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
from queue import Queue
from threading import Thread, Lock

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
        self._creation_timestamp = time_ms()
        message_id, level, l_position, l_reached, r_position, r_reached, state = raw_message.split(':')
        self._message_id, self._level = int(message_id), level
        self._left_distance, self._left_reached = float(l_position)/100.0, True if l_reached == '0' else False
        self._right_distance, self._right_reached = float(r_position)/100.0, True if r_reached == '0' else False
        self._state = stateID2str[int(state)]

    @property
    def creation_timestamp(self):
        return self._creation_timestamp

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
    def left_reached(self):
        return self._left_reached
    
    @property
    def right_distance(self):
        return self._right_distance
    
    @property
    def right_reached(self):
        return self._right_reached
    
    @property
    def state(self):
        return self._state

    def get_delta_distance_output_angle(self, previous_msg, X, Y, THETA):
        new_THETA = THETA
        new_X = X
        new_Y = Y
        delta_distance = 0.0
        if self.left_distance and self.left_distance and previous_msg.left_distance and previous_msg.right_distance: # and self.left_distance * self.right_distance > 0:
            delta_left = (self.left_distance - previous_msg.left_distance)
            delta_right = (self.right_distance - previous_msg.right_distance)

            delta_distance = (delta_left + delta_right) / 2

            theta = (delta_right - delta_left) / ROBOT_WIDTH_M
            if delta_distance != 0:
                distance_traveled_x = math.cos(theta) * delta_distance
                distance_traveled_y = -math.sin(theta) * delta_distance

                new_X = X + (distance_traveled_x * math.cos(THETA) - distance_traveled_y * math.sin(THETA))
                new_Y = Y + (distance_traveled_x * math.sin(THETA) + distance_traveled_y * math.cos(THETA))
            
            if theta != 0:
                new_THETA = theta + THETA

        return new_X, new_Y, new_THETA, delta_distance
        

class RobotPlatform(SerialWrapper):

    def _move_command(self, left_distance, right_distance, time):
        return "G10 {} {} {}".format(left_distance, right_distance, time)

    def __init__(self, serial_dev, baudrate, input_topic, tf2_link_base, tf2_link_child, tf2_rate):
        SerialWrapper.__init__(self, serial_dev, baudrate=baudrate)
        self._tf2_link_base = tf2_link_base
        self._tf2_link_child = tf2_link_child
        self._tf2_wait = tf2_rate
        self._tf_broadcaster = TransformBroadcaster(queue_size=10000)
        rospy.Subscriber(input_topic, PoseStamped, self._goal_callback)

        self._prev_msg = None
        self._last_msg = None
        self._message_lock = Lock()
        self.__threads = [
            Thread(target=self.__process_serial),
            Thread(target=self.__parse_serial),
            Thread(target=self.__health_check),
            Thread(target=self.__track_goal),
            Thread(target=self.__send_tf2)
        ]
        self._incoming_message_queue = Queue()
        self._inc_serial_queue = Queue()
        self._output_tf2_queue = Queue()
        
        self._x, self._y = 0.0, 0.0
        self._current_Pose2d = Pose2D()
        self._current_Pose2d.x = 0.0
        self._current_Pose2d.y = 0.0
        self.last_YAW = 0.0
        self.previous_last_YAW = 0.0
        self.delta_distance = 0.0

        self.__running = False
        self.__wheel_state = -1

        self._tf2_rate = Rate()
        

        while self.__running and rospy.Time.now() != 0:
            rospy.logwarn(f"Client didn't receive time on /time topic yet!")
            time.sleep(1)

    def is_moving(self, delta_distance_limit=0.01, delta_yaw_limit=0.01):
        return self.delta_distance > delta_distance_limit or abs(self.previous_last_YAW - self.last_YAW) > delta_yaw_limit

    def wait_until_stopped(self, delta_distance_limit=0.01, delta_yaw_limit=0.01, timeout=0.0):
        start_waiting_time = round(time.time() * 1000)
        while self.is_moving(delta_distance_limit, delta_yaw_limit):
            if round(time.time() * 1000) - start_waiting_time > timeout * 1000.0:
                return

    def _goal_callback(self, data):
        self._incoming_message_queue.put(data)
    
    def stop(self, *args, **kwargs):
        rospy.loginfo(f"Stopping gracefully.")
        self.__running = False
        for th in self.__threads:
            th.join()

    def __health_check(self):
        while self.__running:
            rospy.loginfo(f"rcv_rate={self.message_rate} emit_rate={self._tf2_rate.mean_rate}")
            time.sleep(1)

    def __send_tf2(self):
        while self.__running:
            transform = self._output_tf2_queue.get()
            self._tf_broadcaster.sendTransform(transform)
            self._output_tf2_queue.task_done()
            self._tf2_rate.update()

    def _parse(self, data):
        rospy.logdebug(f"Received '{data}'")
        self._last_msg = None
        try:
            self._last_msg = MoveMessage(data)
        except Exception as e:
            rospy.logerr(f"Parse error {str(e)}")
        
        if self._last_msg and self._prev_msg and self._last_msg != self._prev_msg:
            self._current_Pose2d.x, self._current_Pose2d.y, self.last_YAW, self.delta_distance = self._last_msg.get_delta_distance_output_angle(self._prev_msg, self._current_Pose2d.x, self._current_Pose2d.y, self.last_YAW)
            
            # if self._last_msg.creation_timestamp - self._prev_msg.creation_timestamp < 30.0:
            platform_transform = TransformStamped()
            platform_transform.header.stamp = rospy.Time.now()
            platform_transform.header.frame_id = self._tf2_link_child
            platform_transform.child_frame_id = self._tf2_link_base # self._tf2_link_child
            platform_transform.transform.translation.x = self._current_Pose2d.x
            platform_transform.transform.translation.y = self._current_Pose2d.y
            platform_transform.transform.translation.z = 0.0
            q = tf_conversions.transformations.quaternion_from_euler(0.0, 0.0, self.last_YAW)
            platform_transform.transform.rotation.x = q[0]
            platform_transform.transform.rotation.y = q[1]
            platform_transform.transform.rotation.z = q[2]
            platform_transform.transform.rotation.w = q[3]
            self._output_tf2_queue.put(platform_transform)
            # else:
            #     rospy.logwarn(f"Dropping tf2 with delay {self._last_msg.creation_timestamp - self._prev_msg.creation_timestamp}>{30.0}")
        self._prev_msg = self._last_msg

    def __track_goal(self):
        while self.__running:
            data = self._incoming_message_queue.get()
            rospy.loginfo(f"Started processing goal {data.pose.position.x} {data.pose.position.y} {data.pose.position.z}")

            dx = data.pose.position.x - self._current_Pose2d.x
            dy = data.pose.position.y - self._current_Pose2d.y

            r, p, y = tf_conversions.transformations.euler_from_quaternion([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])
            
            angle = math.atan2(dy, dx) - self.last_YAW
            if angle > math.pi:
                angle = math.pi - angle
            
            distance = math.sqrt(dx*dx + dy*dy)

            rospy.loginfo(f"Rotating {angle} and moving {distance}")

            turn_distance = (angle * ROBOT_WIDTH_M /2) * 100.0
            turn_time = abs(turn_distance/2)
            turn_command = self._move_command(-turn_distance, turn_distance, turn_time)
            self.write_data(turn_command)
            rospy.loginfo(f"Turning with command '{turn_command}'")
            time.sleep(turn_time/10.0)
            self.wait_until_stopped(0.01, 0.001, turn_time )
            time.sleep(turn_time/10.0)
            move_time = 100.0 * distance / 2
            move_command = self._move_command(distance*100.0, distance*100.0, move_time)
            self.write_data(move_command)
            rospy.loginfo(f"Moving forward with command '{move_command}'")
            time.sleep(move_time/10.0)
            self.wait_until_stopped(0.01, 0.001, move_time)
            time.sleep(move_time/10.0)
            self._incoming_message_queue.task_done()
            rospy.loginfo(f"Finished processing goal {data.pose.position.x} {data.pose.position.y} {data.pose.position.z}")

    def __parse_serial(self):
        while self.__running:
            inc_timestamp, raw_data = self._inc_serial_queue.get()
            current_timestamp = time_ms()
            if current_timestamp - inc_timestamp < 30.0:
                self._parse(raw_data)
            else:
                rospy.logwarn(f"Dropping message with delay {current_timestamp - inc_timestamp}>{30.0}")
            self._inc_serial_queue.task_done()

    def __process_serial(self):
        while self.__running:
            raw_data = self.read_data()
            if raw_data is not None and raw_data != '':
                rospy.logdebug(f"received raw: {raw_data}")
                self._inc_serial_queue.put((time_ms(), raw_data, ))

    def process(self):
        self.__running = True
        for th in self.__threads:
            th.start()

def main():

    rospy.init_node('robot_simplified', log_level=env2log())
    
    serial_dev = rospy.get_param("~serial_dev")
    baudrate = rospy.get_param("~baudrate")
    tf2_rate = rospy.get_param("~tf2_rate")
    input_topic = rospy.get_param("~tf2_input_topic")
    tf2_link_base = rospy.get_param("~tf2_base_link")
    tf2_link_child = rospy.get_param("~tf2_map_link")

    robot = RobotPlatform(serial_dev, baudrate, input_topic, tf2_link_base, tf2_link_child, tf2_rate)
    signal.signal(signal.SIGINT, robot.stop)
    signal.signal(signal.SIGTERM, robot.stop)
    robot.process()