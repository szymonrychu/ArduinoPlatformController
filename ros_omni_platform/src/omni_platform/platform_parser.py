from .serial_helper import ThreadedSerialOutputHandler
from .platform_statics import PlatformStatics

import rospy
import tf
import tf2_ros
import geometry_msgs.msg
import tf_conversions

from threading import Lock
import math


class PlatformParser(ThreadedSerialOutputHandler):

    def parse_serial(self, wheel_id, raw_data):
        rospy.loginfo("{}: {}".format(str(wheel_id+1), raw_data))

class WheelDetails():

    def __init__(self, _id):
        self.__id = _id
        self.__angle_last_position = 0
        self.__angle_error = 0
        self.__angle_last_velocity = 0
        self.__angle_power = 0
        self.__distance_last_position = 0
        self.__distance_error = 0
        self.__distance_last_velocity = 0
        self.__distance_power = 0
        self.__x = 0
        self.__y = 0
        self.__last_read_msg_id = 0
        self.__last_msg_id = 0
        self.__lock = Lock()

    @property
    def id(self):
        return self.__id

    def produce_tf(self):
        with self.__lock:
            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = f"/wheel{self.__id}_axis"
            t.child_frame_id = f"/wheel{self.__id}"
            t.transform.translation.x = self.__x
            t.transform.translation.y = self.__y
            t.transform.translation.z = 0.0
            if self.__id in [1, 2]:
                q = tf_conversions.transformations.quaternion_from_euler(0, self.__distance_last_position/200.0, math.pi + math.pi/2 - self.__angle_last_position)
            else:
                q = tf_conversions.transformations.quaternion_from_euler(0, self.__distance_last_position/200.0, math.pi + math.pi/2 + self.__angle_last_position)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            return t
    
    def parse(self, raw_data):
        try:
            data1, data2, timeDelta = raw_data.split(' ')
            msg_id, msg_level, ang_last_pos, ang_err, ang_last_vel, ang_p = data1.split(':')
            dst_last_pos, dst_err, dst_last_vel, dst_p = data2.split(':')
            with self.__lock:
                self.__angle_last_position = float(ang_last_pos)
                self.__angle_error = float(ang_err)
                self.__angle_last_velocity = float(ang_last_vel)
                self.__angle_power = float(ang_p)
                current_distance = float(dst_last_pos)
                self.__distance_error = float(dst_err)
                self.__distance_last_velocity = float(dst_last_vel)
                self.__distance_power = float(dst_p)
                distance_delta = current_distance - self.__distance_last_position
                self.__x += distance_delta * math.cos(self.__angle_last_position)
                self.__y += distance_delta * math.sin(self.__angle_last_position)
                self.__distance_last_position = current_distance
                self.__last_msg_id = int(msg_id)
            rospy.logdebug(f"Parsed: {raw_data}")
        except ValueError:
            rospy.loginfo(f"Error Parsing: {raw_data}")

    @property
    def new_data_available(self):
        return not self.__last_msg_id == self.__last_read_msg_id

    def data_read(self):
        self.__last_read_msg_id = self.__last_msg_id


class PlatformStateTransformPublisher(ThreadedSerialOutputHandler):

    def __init__(self, data_queue):
        ThreadedSerialOutputHandler.__init__(self, data_queue)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster()
        self._wheels = []
        for c in range(PlatformStatics.WHEEL_NUM):
            self._wheels.append(WheelDetails(c))
        rospy.Timer(rospy.Duration(0, 10), self.send_transforms)

    def send_transforms(self, event=None):
        for wheel in self._wheels:
            self._tf_broadcaster.sendTransform(wheel.produce_tf())
            

    def parse_serial(self, wheel_id, raw_data):
        self._wheels[wheel_id].parse(raw_data)
        rospy.loginfo(f"W_{wheel_id}: {raw_data}")
        self._tf_broadcaster.sendTransform(self._wheels[wheel_id].produce_tf())
        
        # wheel_positions = (0, 0, 0)
        # wheel_positions = [sum(x) for x in zip(wheel_positions, wheel.current_translation)]
        # wheel.data_read()
        # self._tf_broadcaster.sendTransform(wheel_positions, (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), 'ignite_robot', f"/base")
        
        
