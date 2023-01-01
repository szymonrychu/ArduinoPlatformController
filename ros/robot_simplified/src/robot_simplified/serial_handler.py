from enum import Enum
import rospy
import serial
import traceback
import traceback
import time
import math
import signal
import tf_conversions
import numpy as np
from threading import Thread, Lock
import tf2_ros

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import numpy as np
import queue

from dataclasses import dataclass

from geometry_msgs.msg import Quaternion, PoseStamped, TransformStamped, Vector3
from sensor_msgs.msg import Imu
from std_srvs.srv import SetBool, Trigger, TriggerRequest
from .lib import Rate, Latch, RobotQuaternion, env2log, ROBOT_WIDTH_M, time_ms

class MessageState(Enum):
    WHEEL_STATE_FRESH           = 0
    STATE_MOVING_FORWARD        = 1
    STATE_MOVING_BACKWARD       = 2
    STATE_BUSY_MOVING_TURNING   = 4

@dataclass
class ReceivedMessage:
    message_id: int
    level: str
    state: int
    left_reached: bool
    right_reached: bool
    delta_left: float
    delta_right: float
    qW: float
    qX: float
    qY: float
    qZ: float
    angular_velocity_x: float
    angular_velocity_y: float
    angular_velocity_z: float
    linear_acceleration_x: float
    linear_acceleration_y: float
    linear_acceleration_z: float


class SerialWrapper():

    def __init__(self, fpath, baudrate=115200):
        self._fpath = fpath
        try:
            self.serial = serial.Serial(fpath, baudrate, timeout=0.1)
        except Exception:
            tb = traceback.format_exc()
            rospy.loginfo(str(tb))
        self._rate = Rate()

    @property
    def message_rate(self):
        return self._rate.mean_rate

    def data_available(self):
        return self.serial.inWaiting()

    def read_data(self):
        raw_data = None
        try:
            if self.data_available():
                raw_line = self.serial.readline()
                if raw_line is not None:
                    raw_data = raw_line.decode('ascii')
                    if raw_data != '':
                        if raw_data[-1] == '\n':
                            raw_data = raw_data[:-1]
                    self._rate.update()
            rospy.logdebug(f"Reading from serial: '{raw_data}'")
        except serial.SerialTimeoutException:
            pass
        except UnicodeDecodeError:
            rospy.logwarn('cannot parse "{}"'.format(raw_data))
            self.repair_serial()
        return raw_data

    def write_data(self, raw_data):
        try:
            rospy.loginfo(f"Writing to serial: '{raw_data}'")
            self.serial.write('{}\n'.format(raw_data).encode())
            return True
        except TypeError:
            tb = traceback.format_exc()
            rospy.loginfo(str(tb))
            self._repair_serial()

    def repair_serial(self):
        try:
            self.serial.close()
            self.serial = None
        except Exception:
            tb = traceback.format_exc()
            rospy.loginfo(str(tb))
        SerialWrapper.__init__(self, self._fpath, self.__baudrate)

class RobotSerialHandler(SerialWrapper):

    ROBOT_STATE_READY         = 'ready'
    STATE_MOVING_FORWARD      = 'moving_forward'
    STATE_MOVING_BACKWARD     = 'moving_backward'
    STATE_BUSY_MOVING_TURNING = 'moving_turning'

    stateID2str = {
        0: ROBOT_STATE_READY,
        1: STATE_MOVING_FORWARD,
        2: STATE_MOVING_BACKWARD,
        3: STATE_BUSY_MOVING_TURNING
    }

    def __init__(self):
        rospy.init_node('robot_serial_handler', log_level=env2log())
        self.__running = True
        self.__last_odometry = None
        self.__odometry_lock = Lock()

        odometry_output_queue_size = rospy.get_param("~odometry_output_queue_size")
        odometry_output_topic_name = rospy.get_param("~odometry_output_topic_name")
        self._odometry_pub = rospy.Publisher(odometry_output_topic_name, Odometry, queue_size=odometry_output_queue_size)

        imu_queue_size = rospy.get_param("~imu_output_queue_size")
        imu_topic_name = rospy.get_param("~imu_output_topic_name")
        self._imu_pub = rospy.Publisher(imu_topic_name, Imu, queue_size=imu_queue_size)


        pose_delta_input_topic_name = rospy.get_param("~pose_delta_input_topic_name")
        rospy.Subscriber(pose_delta_input_topic_name, PoseStamped, self._input_pose_delta_handler)


        self._link_base = rospy.get_param("~base_link")
        self._link_child = rospy.get_param("~map_link")

        self.__tf2_broadcaster = tf2_ros.TransformBroadcaster()

        serial_dev = rospy.get_param("~serial_dev")
        baudrate = rospy.get_param("~baudrate")
        SerialWrapper.__init__(self, serial_dev, baudrate)


        while self.__running and rospy.Time.now() == 0:
            rospy.logwarn(f"Client didn't receive time on /time topic yet!")
            time.sleep(1)
        self.reset_hector_mapping()

    def _input_pose_delta_handler(self, data: PoseStamped):
        pass

    def stop(self, *args, **kwargs):
        rospy.loginfo(f"Stopping gracefully.")
        self.__running = False
    
    def __parse_raw_message(self, raw_message: str):
        rospy_Time_now = rospy.Time.now()
        # 0000002544:INF:0:0:0:0.0000:0.0000:0.9979:0.0190:-0.0615:-0.0018:0.0000:0.0175:-0.0033:-0.1600:0.0400:-0.6200
        message_id, level, \
            state, left_reached, right_reached, delta_left, delta_right, \
            qW, qX, qY, qZ, gX, gY, gZ, aX, aY, aZ = raw_message.split(':')
        message_id = int(message_id)
        raw_state_map = {
            0: 'READY',
            1: 'MOVING_FORWARD',
            2: 'MOVING_BACKWARD',
            3: 'TURNING',
        }
        state = raw_state_map[state]
        left_reached, right_reached = left_reached == 0, right_reached == 0
        delta_left, delta_right = float(delta_left), float(delta_right)

        imu = Imu()
        imu.header.stamp = rospy_Time_now
        imu.orientation = Quaternion(float(qW), float(qX), float(qY), float(qZ))
        imu.orientation_covariance[0] = 1e-9
        imu.orientation_covariance[4] = 1e-9
        imu.orientation_covariance[8] = 1e-9
        imu.angular_velocity = Vector3(float(gX), float(gY), float(gZ))
        imu.angular_velocity_covariance[0] = 1e-9
        imu.angular_velocity_covariance[4] = 1e-9
        imu.angular_velocity_covariance[8] = 1e-9
        imu.linear_acceleration = Vector3(float(aX), float(aY), float(aZ))
        imu.linear_acceleration_covariance[0] = 1e-9
        imu.linear_acceleration_covariance[4] = 1e-9
        imu.linear_acceleration_covariance[8] = 1e-9

        pose_stamped = PoseStamped()

        return imu, pose_stamped

            
        _ints = [ 0, 2 ]
        _strings = [ 1 ] # level
        _bools = [3, 4] # id, state, left_reached, right_reached
        _floats = [5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16] # delta_L, delta_R, qw, qx, qy, qz, agVx, agVy, agVz, lAcx, lAcy, lAcz
        try:
            splitted_string = raw_message.split(':')
            result = tuple()
            for id, item in enumerate(splitted_string):
                if id in _ints:
                    result = result + (int(item), )
                elif id in _strings:
                    result = result + (item, )
                elif id in _bools:
                    result = result + (True if item == '0' else False, )
                elif id in _floats:
                    result = result + (float(item), )
            if len(result) - (len(_ints) + len(_strings) + len(_bools) + len(_floats)) != 0:
                raise ValueError()
            msg = ReceivedMessage(*result)
            return msg
        except ValueError:
            rospy.logwarn(f"Can't parse message: {raw_message}")
        except AttributeError:
            pass
        return None

    def _send_goal_reached(self, state: bool):
        b = Bool()
        b.data = state
        self._goal_reached_pub.publish(b)

    def __odometry_callback(self, data):
        with self.__odometry_lock:
            self.__last_odometry = data

    def __wait_for_goal_with_timeout(self, timeout, loop_time=0.1):
        for _ in range(round(timeout/loop_time)):
            time.sleep(loop_time)
            if self.__goal_reached:
                rospy.loginfo("Wait done, goal reached")
                return
        rospy.logwarn("Timeout reached")

    def _goal_callback(self, data):
        last_odometry, dx, dy = None, 0, 0

        with self.__odometry_lock:
            if not self.__last_odometry:
                return
            last_odometry = self.__last_odometry


        dx = data.pose.position.x - last_odometry.pose.pose.position.x
        dy = data.pose.position.y - last_odometry.pose.pose.position.y
        R, P, Y = tf_conversions.transformations.euler_from_quaternion(last_odometry.pose.pose.orientation.w, last_odometry.pose.pose.orientation.x, last_odometry.pose.pose.orientation.y, last_odometry.pose.pose.orientation.z)


        angle = math.atan2(dy, dx) - Y
        if angle > math.pi:
            angle = math.pi - angle
        if angle < -math.pi:
            angle = -math.pi + angle

        distance = math.sqrt(dx*dx + dy*dy)
        

        if angle != 0:
            turn_distance = (angle * ROBOT_WIDTH_M /2)
            turn_time = abs(turn_distance/4)*100.0
            l, r, t = -turn_distance, turn_distance, turn_time
            move_command = f"G10 {l} {r} {t}"
            self.write_data(move_command)
        
            rospy.loginfo(f"Rotating {angle} over {turn_time}")
            time.sleep(turn_time/10.0)
            self.__wait_for_goal_with_timeout((turn_time - turn_time/10.0))
            rospy.loginfo(f"Done rotating {angle}")

        if distance != 0:
            move_time = abs(distance/4)*100.0
            l, r, t = distance, distance, move_time
            move_command = f"G10 {l} {r} {t}"
            self.write_data(move_command)

            rospy.loginfo(f"Moving {distance} over {move_time}")
            time.sleep(move_time/10.0)
            self.__wait_for_goal_with_timeout((move_time - move_time/10.0))
            rospy.loginfo(f"Done moving {distance}")


    def process(self):
        self.__running = True
        while self.__running:
            rospy_header_timestamp = rospy.Time.now()

            message = self.__parse_raw_message(self.read_data())
            if message:
                odometry = Odometry()
                t = TransformStamped()

                odometry.header.stamp = rospy_header_timestamp
                odometry.header.frame_id = self._link_child
                odometry.child_frame_id = self._link_base
                odometry.pose.pose.orientation = Quaternion(message.qW, message.qX, message.qY, message.qZ)

                R, P, Y = tf_conversions.transformations.euler_from_quaternion(message.qW, message.qX, message.qY, message.qZ)

                delta_translation = (message.delta_right + message.delta_left) / 2

                dx = delta_translation * math.cos(Y)
                dy = delta_translation * math.sin(Y)

                with self.__odometry_lock:
                    odometry.pose.pose.position.x = dx + self.__last_odometry.pose.pose.position.x
                    odometry.pose.pose.position.y = dy + self.__last_odometry.pose.pose.position.y

                t.header = odometry.header
                t.transform.translation.x = odometry.pose.pose.position.x
                t.transform.translation.y = odometry.pose.pose.position.y
                t.transform.translation.z = odometry.pose.pose.position.z

                t.transform.rotation.x = odometry.pose.pose.orientation.x
                t.transform.rotation.y = odometry.pose.pose.orientation.y
                t.transform.rotation.z = odometry.pose.pose.orientation.z
                t.transform.rotation.w = odometry.pose.pose.orientation.w
                
                self._odometry_pub.publish(odometry)
                self._send_goal_reached(abs(delta_translation) < 0.01)
                self.__tf2_broadcaster.sendTransform(t)

                with self.__odometry_lock:
                    self.__last_odometry = odometry
        
            time.sleep(0.001)

def main():
    handler = RobotSerialHandler()
    signal.signal(signal.SIGINT, handler.stop)
    signal.signal(signal.SIGTERM, handler.stop)
    handler.process()