import rospy
import serial
import traceback
import traceback
import time
import math
import signal
import tf_conversions

from geometry_msgs.msg import Point, Pose, Quaternion
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
        output_topic_name = rospy.get_param("~output_topic")
        output_topic_queue_size = rospy.get_param("~output_topic_queue_size")
        self.__hector_pause_mapping_sname = rospy.get_param("~hector_mapping_pause_sname")
        self.__hector_reset_mapping_sname = rospy.get_param("~hector_mapping_reset_sname")
        self._output_pub = rospy.Publisher(output_topic_name, Pose, queue_size=output_topic_queue_size)
        rospy.Subscriber(input_topic_name, Point, self._input_callback)
        SerialWrapper.__init__(self, serial_dev, baudrate)
        self.__pose = Pose()
        self.__running = True
        self.__suppresed_log = SupressedLog(1)
        self.__incoming_serial_rate = Rate()

        self.__deltas_initialized = False
        self.__previous_left_distance = 0.0
        self.__previous_right_distance = 0.0

        self.__delta_distance = 0.0
        self.__q = RobotQuaternion()
        self.__previous_q = RobotQuaternion()

        self.__primed = False

        while self.__running and rospy.Time.now() == 0:
            rospy.logwarn(f"Client didn't receive time on /time topic yet!")
            time.sleep(1)
        self.reset_hector_mapping()

    def is_moving(self, delta_distance_limit=0.01, delta_yaw_limit=0.0001):
        r, p, y = tf_conversions.transformations.euler_from_quaternion(self.__delta_q.to_arr())
        dD, dR, dP, dY = abs(self.__delta_distance), abs(r), abs(p), abs(y)
        result = dD > delta_distance_limit or dY > delta_yaw_limit #  or dR > delta_yaw_limit or dP > delta_yaw_limit
        if result:
            rospy.loginfo(f"Moving [dD,dR,dP,dY] [{dD},{dR},{dP},{dY}]")
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

    def process(self):
        self.__running = True

        while self.__running:
            raw_data = self.read_data()

            if raw_data:
                try:
                    message_id, level, l_position, l_reached, r_position, r_reached, state, w, x, y, z, temp = raw_data.split(':')

                    message_id, level, state = int(message_id), level, RobotSerialHandler.stateID2str[int(state)]
                    left_distance, left_reached = float(l_position)/100.0, True if l_reached == '0' else False
                    right_distance, right_reached = float(r_position)/100.0, True if r_reached == '0' else False

                    
                    self.__q = RobotQuaternion(float(x), float(y), float(z), float(w))
                    
                    if self.__primed:
                        r, p, y = tf_conversions.transformations.euler_from_quaternion(self.__q.to_arr())
                            
                        delta_left = left_distance - self.__previous_left_distance
                        delta_right = right_distance - self.__previous_right_distance
                        self.__delta_distance = (delta_left + delta_right) / 2

                        pq_inv = RobotQuaternion.from_arr(tf_conversions.transformations.quaternion_inverse(self.__previous_q.to_arr()))

                        self.__delta_q = RobotQuaternion.from_arr(tf_conversions.transformations.quaternion_multiply(self.__q.to_arr(), pq_inv.to_arr()))
                        
                        dr, dp, dy = tf_conversions.transformations.euler_from_quaternion(self.__delta_q.to_arr())

                        # self.__delta_theta = (delta_right - delta_left) / ROBOT_WIDTH_M

                        distance_traveled_x = math.cos(dy) * self.__delta_distance
                        distance_traveled_y = -math.sin(dy) * self.__delta_distance

                        delta_x = distance_traveled_x * math.cos(y) - distance_traveled_y * math.sin(y)
                        delta_y = distance_traveled_x * math.sin(y) + distance_traveled_y * math.cos(y)

                        self.__pose.position.x += delta_x
                        self.__pose.position.y += delta_y
                        self.__pose.orientation = self.__q.q()

                        self.__incoming_serial_rate.update()
                        # rospy.loginfo(f"Last Pose XYZ=[{self.__pose.position.x},{self.__pose.position.y},{self.__pose.position.y}] RPY=[{r},{p},{y}] ({self.__incoming_serial_rate.mean_rate}ms)")
                        # self.__suppresed_log.handler(rospy.loginfo, f"Last Pose XYZ=[{self.__pose.position.x},{self.__pose.position.y},{self.__pose.position.y}] RPY=[{r},{p},{y}] ({self.__incoming_serial_rate.mean_rate}ms)")
                        
                        self._output_pub.publish(self.__pose)
                        
                    self.__previous_q = self.__q
                    self.__previous_left_distance = left_distance
                    self.__previous_right_distance = right_distance
                    self.__primed = True
                except ValueError:
                    rospy.logwarn(f"Unparsable '{raw_data}'")
                # except AttributeError:
                #     pass
            
            time.sleep(0.001)

def main():
    handler = RobotSerialHandler()
    signal.signal(signal.SIGINT, handler.stop)
    signal.signal(signal.SIGTERM, handler.stop)
    handler.process()