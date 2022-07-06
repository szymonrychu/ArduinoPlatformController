import rospy
import serial
import traceback
import traceback
import time
import math
import signal
import tf_conversions
import numpy as np

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from geometry_msgs.msg import Point, Pose2D, Quaternion
from std_srvs.srv import SetBool, Trigger, TriggerRequest

from .lib import Rate, SupressedLog, RobotQuaternion, env2log, ROBOT_WIDTH_M

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
        except serial.SerialTimeoutException:
            pass
        except UnicodeDecodeError:
            rospy.logwarn('cannot parse "{}"'.format(raw_data))
            self.repair_serial()
        return raw_data

    def write_data(self, raw_data):
        try:
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
        serial_dev = rospy.get_param("~serial_dev")
        baudrate = rospy.get_param("~baudrate")
        input_topic_name = rospy.get_param("~input_topic")
        self.__hector_pause_mapping_sname = rospy.get_param("~hector_mapping_pause_sname")
        self.__hector_reset_mapping_sname = rospy.get_param("~hector_mapping_reset_sname")

        output_topic_name = rospy.get_param("~output_topic")
        output_topic_queue_size = rospy.get_param("~output_topic_queue_size")
        self._output_pub = rospy.Publisher(output_topic_name, Pose2D, queue_size=output_topic_queue_size)

        odometry_queue_size = rospy.get_param("~odometry_queue_size")
        odometry_topic_name = rospy.get_param("~odometry_topic_name")
        self._odometry_pub = rospy.Publisher(odometry_topic_name, Odometry, queue_size=odometry_queue_size)

        imu_queue_size = rospy.get_param("~imu_queue_size")
        imu_topic_name = rospy.get_param("~imu_topic_name")
        self._imu_pub = rospy.Publisher(imu_topic_name, Imu, queue_size=imu_queue_size)

        self._link_base = rospy.get_param("~base_link")
        self._link_child = rospy.get_param("~map_link")

        rospy.Subscriber(input_topic_name, Point, self._input_callback)
        SerialWrapper.__init__(self, serial_dev, baudrate)
        self.__running = True
        self.__theta = 0.0
        self.__x = 0.0
        self.__y = 0.0
        self.__delta_left = 0.0
        self.__delta_right = 0.0

        while self.__running and rospy.Time.now() == 0:
            rospy.logwarn(f"Client didn't receive time on /time topic yet!")
            time.sleep(1)
        self.reset_hector_mapping()

    def is_moving(self, delta_distance_limit=0.001):
        result = abs(self.__delta_left) > delta_distance_limit or abs(self.__delta_right) > delta_distance_limit
        if result:
            rospy.loginfo(f"Moving [dX,dY] [{self.__delta_x},{self.__delta_y}]")
        return result

    def reset_hector_mapping(self):
        try:
            rospy.loginfo(f"Triggering Hector mapping service's reset '{self.__hector_reset_mapping_sname}'")
            hector_mapping_service = rospy.ServiceProxy(self.__hector_reset_mapping_sname, Trigger)
            return hector_mapping_service(TriggerRequest())
        except rospy.ServiceException as e:
            rospy.logwarn(f"Hector mapping service '{self.__hector_reset_mapping_sname}' call failed: {e}")

    def toggle_hector_mapping(self, state_on=True):
        try:
            rospy.loginfo(f"Toggling Hector mapping service '{self.__hector_pause_mapping_sname}' with value '{'true' if state_on else 'false'}'")
            hector_mapping_service = rospy.ServiceProxy(self.__hector_pause_mapping_sname, SetBool)
            return hector_mapping_service(state_on)
        except rospy.ServiceException as e:
            rospy.logwarn(f"Hector mapping service '{self.__hector_pause_mapping_sname}' call failed: {e}")

    def _input_callback(self, data):
        # self.toggle_hector_mapping(False)
        left_distance, right_distance, time_s = data.x, data.y, data.z
        move_command = "G10 {} {} {}".format(left_distance, right_distance, time_s)
        self.write_data(move_command)
        rospy.loginfo(f"Waiting for robot to finish command '{move_command}'")
        while True:
            time.sleep(time_s/2.0)
            if not self.is_moving():
                break
        rospy.loginfo(f"Command '{move_command}' finished")
        self.reset_hector_mapping()
        # self.toggle_hector_mapping(True)

    def stop(self, *args, **kwargs):
        rospy.loginfo(f"Stopping gracefully.")
        self.__running = False
    
    def __parse_raw_message(self, raw_message):
        _ints = [ 0 ]
        _strings = [ 1 ] # level
        _bools = [2, 3, 4] # id, state, left_reached, right_reached
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
            result += (True, )
            return result
        except ValueError:
            rospy.logwarn(f"Unparsable '{raw_message}'")
        except AttributeError:
            pass
        tmp = tuple([None] * (len(_ints) + len(_strings) + len(_bools) + len(_floats)))
        tmp += (False,)
        return tmp

    def process(self):
        self.__running = True
        while self.__running:
            m_id, level, state, l_reached, r_reached, \
                self.__delta_left, self.__delta_right, \
                qW, qX, qY, qZ, \
                angVX, angVY, angVZ, \
                lAccX, lAccY, lAccZ, success = self.__parse_raw_message(self.read_data())
            
            if success:
                # based on https://github.com/jfstepha/differential-drive/blob/master/scripts/diff_tf.py
                # and on: http://docs.ros.org/en/melodic/api/robot_localization/html/preparing_sensor_data.html
                # and on: https://github.com/Sollimann/CleanIt/blob/main/autonomy/src/slam/README.md
                ds = (self.__delta_right + self.__delta_left) / 2
                ds_2b = ds/ (2*ROBOT_WIDTH_M)
                dyaw = (self.__delta_right - self.__delta_left) / ROBOT_WIDTH_M
                c_yaw = math.cos(self.__theta + (dyaw / 2.0))
                s_yaw = math.sin(self.__theta + (dyaw / 2.0))

                dx = ds * c_yaw
                dy = ds * s_yaw

                # p_old = np.array([self.__x, self.__y, self.__theta])
                # delta = np.array([dx, dy, dyaw])
                # p_new = np.add(p_old, delta)

                self.__theta += dyaw
                self.__x += dx
                self.__y += dy

                odom_q = Quaternion()
                odom_q.x = 0.0
                odom_q.y = 0.0
                odom_q.z = math.sin(self.__theta / 2)
                odom_q.w = math.cos(self.__theta / 2)

                odometry = Odometry()
                odometry.header.stamp = rospy.Time.now()
                odometry.header.frame_id = self._link_child
                odometry.child_frame_id = self._link_base
                odometry.pose.pose.orientation = odom_q
                odometry.pose.pose.position.x = self.__x
                odometry.pose.pose.position.y = self.__y
                odometry.pose.covariance = np.array(   [1,   0,   0,   0,   0,   0,
                                                        0,   1,   0,   0,   0,   0,
                                                        0,   0,   1,   0,   0,   0,
                                                        0,   0,   0,   1,   0,   0,
                                                        0,   0,   0,   0,   1,   0,
                                                        0,   0,   0,   0,   0,   .7])**2

                # kr, kl = 1.0, 1.0
                # i_00, i_11 = kr * abs(self.__delta_left), kl * abs(self.__delta_right)
                # vel_covar = np.matrix([[i_00, 0],[0, i_11]])

                # i_00 = (c_yaw / 2.0) - (ds_2b * s_yaw)
                # i_01 = (c_yaw / 2.0) + (ds_2b * s_yaw)
                # i_10 = (s_yaw / 2.0) + (ds_2b * c_yaw)
                # i_11 = (s_yaw / 2.0) - (ds_2b * c_yaw)
                # i_20 = 1.0 / ROBOT_WIDTH_M
                # i_21 = -i_20
                # vel_jacob = np.matrix([[i_00, i_01], [i_10, i_11], [i_20, i_21]])

                # vel_covar_matrix_est = vel_jacob.dot(vel_covar).dot(vel_jacob.transpose())

                # i_00 = 1.0
                # i_01 = 0.0
                # i_02 = -dy
                # i_10 = 0.0
                # i_11 = 1.0
                # i_12 = dx
                # i_20 = 0.0
                # i_21 = 0.0
                # i_22 = 1.0
                # pose_jacob = np.matrix([[i_00, i_01, i_02], [i_10, i_11, i_12], [i_20, i_21, i_22]])

                # pose_covar 


                imu = Imu()
                imu.orientation = Quaternion(qW, qX, qY, qZ)
                imu.angular_velocity.x = angVX
                imu.angular_velocity.y = angVY
                imu.angular_velocity.z = angVZ
                imu.linear_acceleration.x = lAccX
                imu.linear_acceleration.y = lAccY
                imu.linear_acceleration.z = lAccZ

                pose2d = Pose2D()
                pose2d.x = self.__x
                pose2d.y = self.__y
                pose2d.theta = self.__theta

                self._imu_pub.publish(imu)
                self._odometry_pub.publish(odometry)
                self._output_pub.publish(pose2d)
            
            time.sleep(0.001)

def main():
    handler = RobotSerialHandler()
    signal.signal(signal.SIGINT, handler.stop)
    signal.signal(signal.SIGTERM, handler.stop)
    handler.process()