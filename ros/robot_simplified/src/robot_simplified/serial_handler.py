import rospy
import serial
import traceback
import traceback
import time
import math
import signal

from geometry_msgs.msg import Point, Pose2D
from std_srvs.srv import SetBool, Trigger, TriggerRequest

from .lib import Rate, SupressedLog, env2log, ROBOT_WIDTH_M

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
            rospy.loginfo('cannot parse "{}"'.format(raw_data))
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
        self._output_pub = rospy.Publisher(output_topic_name, Pose2D, queue_size=output_topic_queue_size)
        rospy.Subscriber(input_topic_name, Point, self._input_callback)
        SerialWrapper.__init__(self, serial_dev, baudrate)
        self.__pose = Pose2D()
        self.__running = True
        self.__suppresed_log = SupressedLog(1)
        self.__incoming_serial_rate = Rate()

        self.__deltas_initialized = False
        self.__previous_left_distance = None
        self.__previous_right_distance = None

        self.__delta_distance = 0.0
        self.__delta_theta = 0.0

        while self.__running and rospy.Time.now() == 0:
            rospy.logwarn(f"Client didn't receive time on /time topic yet!")
            time.sleep(1)
        self.reset_hector_mapping()

    def is_moving(self, delta_distance_limit=0.01, delta_yaw_limit=0.001):
        return abs(self.__delta_distance) > delta_distance_limit or abs(self.__delta_theta) > delta_yaw_limit

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
        while True:
            time.sleep(time_s/10.0)
            if not self.is_moving():
                break
        self.reset_hector_mapping()
        # self.toggle_hector_mapping(True)

    def stop(self, *args, **kwargs):
        rospy.loginfo(f"Stopping gracefully.")
        self.__running = False

    def process(self):
        self.__running = True

        while self.__running:
            raw_data = self.read_data()
            try:
                message_id, level, l_position, l_reached, r_position, r_reached, state = raw_data.split(':')

                message_id, level = int(message_id), level
                left_distance, left_reached = float(l_position)/100.0, True if l_reached == '0' else False
                right_distance, right_reached = float(r_position)/100.0, True if r_reached == '0' else False

                state = RobotSerialHandler.stateID2str[int(state)]

                if self.__previous_left_distance and self.__previous_right_distance:
                    delta_left = left_distance - self.__previous_left_distance
                    delta_right = right_distance - self.__previous_right_distance

                    self.__delta_distance = (delta_left + delta_right) / 2
                    self.__delta_theta = (delta_right - delta_left) / ROBOT_WIDTH_M

                    distance_traveled_x = math.cos(self.__delta_theta) * self.__delta_distance
                    distance_traveled_y = -math.sin(self.__delta_theta) * self.__delta_distance

                    delta_x = distance_traveled_x * math.cos(self.__pose.theta) - distance_traveled_y * math.sin(self.__pose.theta)
                    delta_y = distance_traveled_x * math.sin(self.__pose.theta) + distance_traveled_y * math.cos(self.__pose.theta)

                    self.__pose.x += delta_x
                    self.__pose.y += delta_y
                    self.__pose.theta += self.__delta_theta

                    self.__incoming_serial_rate.update()
                    self.__suppresed_log.handler(rospy.loginfo, f"Last Pose2D [{self.__pose.x}:{self.__pose.y}:{self.__pose.theta}] ({self.__incoming_serial_rate.mean_rate}ms)")
                    
                    self._output_pub.publish(self.__pose)
                
                self.__previous_left_distance = left_distance
                self.__previous_right_distance = right_distance
            except ValueError:
                rospy.logwarn(f"Unparsable '{raw_data}'")
            except AttributeError:
                pass
            
            time.sleep(0.001)

def main():
    handler = RobotSerialHandler()
    signal.signal(signal.SIGINT, handler.stop)
    signal.signal(signal.SIGTERM, handler.stop)
    handler.process()