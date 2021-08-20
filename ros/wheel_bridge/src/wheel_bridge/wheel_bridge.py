from .serial_helper import SerialWrapper
import rospy
from geometry_msgs.msg import Vector3, TransformStamped
import tf
import tf_conversions
import tf2_ros
import signal
import math

class Wheel(SerialWrapper):

    def _move_command(self, distance, angle, time):
        return "G11 {} {} {}".format(distance, angle, time)

    def __init__(self, serial_dev, baudrate, input_topic, output_topic, tf2_base_link, tf2_output):
        SerialWrapper.__init__(self, serial_dev, baudrate=baudrate)
        self.__running = False
        self.__last_distance = 0.0
        self.__distance_set = False
        self._tf_broadcaster = tf2_ros.TransformBroadcaster()
        self._tf2_base_link = tf2_base_link
        self._tf2_output = tf2_output
        rospy.Subscriber(input_topic, Vector3, self._topic_callback)
        self._output_pub = rospy.Publisher(output_topic, TransformStamped, queue_size=10)
        self._x, self._y = 0.0, 0.0

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
        try:
            # 35340:INF:-0.0204:0.0204:-0.0000:-76 0.0001:-0.0001:-0.0000:0 0.114
            data1, data2, timeDelta = data.split(' ')
            msg_id, msg_level, ang_last_pos, ang_err, ang_last_vel, ang_p = data1.split(':')
            dst_last_pos, dst_err, dst_last_vel, dst_p = data2.split(':')

            current_distance = float(dst_last_pos)
            current_distance_v = float(dst_last_vel)
            current_angle = float(ang_last_pos) * 2.0
            current_angle_v = float(ang_last_vel)
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
            self._output_pub.publish(self._xyzRPY2TransformStamped(self._x, self._y, 0, 0, 0, current_angle))
            self.__last_distance = current_distance
        except ValueError:
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
    rospy.init_node('wheel_bridge', anonymous=True)

    serial_dev = rospy.get_param("~serial_dev")
    baudrate = rospy.get_param("~baudrate")
    input_topic = rospy.get_param("~input_topic")
    output_topic = rospy.get_param("~output_topic")
    tf2_base_link = rospy.get_param("~tf2_base_link")
    tf2_output = rospy.get_param("~tf2_output")

    wheel = Wheel(serial_dev, baudrate, input_topic, output_topic, tf2_base_link, tf2_output)

    signal.signal(signal.SIGINT, wheel.stop)
    Wheel(serial_dev, baudrate, input_topic, output_topic, tf2_base_link, tf2_output).process()