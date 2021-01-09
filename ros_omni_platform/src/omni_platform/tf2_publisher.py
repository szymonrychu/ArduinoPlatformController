
import rospy
import tf
import tf2_ros
import geometry_msgs.msg
import tf_conversions

import math

from .platform_statics import PlatformStatics
from .serial_helper import ThreadedSerialOutputHandler
import threading

class TF2BaseLink():

    def __init__(self, part_name):
        self.__part_name = part_name

    @property
    def link_name(self):
        return self.__part_name

    def update(self, x, y, z, R, P, Y):
        pass

class TF2Link(TF2BaseLink):

    def __init__(self, part_name, root_part, x=0, y=0, z=0, R=0, P=0, Y=0):
        TF2BaseLink.__init__(self, part_name)
        self.__root_part = root_part
        self.__prevX = x
        self.__prevY = y
        self.__prevZ = z
        self.__prevR = R
        self.__prevP = P
        self.__prevY = Y

    @property
    def root(self):
        return self.__root_part

    def update(self, x, y, z, R, P, Y, increment=True):
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.__root_part.link_name
        t.child_frame_id = self.link_name
        t.transform.translation.x = self.__prevX + x
        t.transform.translation.y = self.__prevY + y
        t.transform.translation.z = self.__prevZ + z
        if increment:
            self.__prevX = t.transform.translation.x
            self.__prevY = t.transform.translation.y
            self.__prevZ = t.transform.translation.z
        q = tf_conversions.transformations.quaternion_from_euler(\
            self.__prevR + R, self.__prevP + P, self.__prevY + Y)
        if increment:
            self.__prevR = self.__prevR + R
            self.__prevP = self.__prevP + P
            self.__prevY = self.__prevY + Y
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        return t
        
class TF2WheelWithPivot(TF2BaseLink):

    def __init__(self, wheel_id, base_link, x, y, z, base_wheel_prefix='/base_wheel_', wheel_prefix='/wheel_'):
        TF2BaseLink.__init__(self, f"{wheel_prefix}{wheel_id}")
        self.__wheel_id = wheel_id
        self.__prev_distance = 0
        self.__last_msg_id = 0
        self.__x, self.__y, self.__z = 0, 0, 0
        self.__dx, self.__dy, self.__dz = 0, 0, 0
        self.__wheel_pivot = TF2Link(f"{base_wheel_prefix}{wheel_id}", base_link, x=x, y=y, z=z)
        self.__wheel = TF2Link(f"{wheel_prefix}{wheel_id}", self.__wheel_pivot)

    @property
    def xyz(self):
        return self.__x, self.__y, self.__z

    @property
    def delta_xyz(self):
        return self.__dx, self.__dy, self.__dz

    def update_base_wheel(self):
        return self.__wheel_pivot.update(0, 0, 0, 0, 0, 0)
    
    def update(self, x, y, z, R, P, Y):
        return self.__wheel.update(0, 0, 0, R, P, Y, increment=False)

    def parse_wheel(self, raw_data):
        try:
            data1, data2, timeDelta = raw_data.split(' ')
            msg_id, msg_level, ang_last_pos, ang_err, ang_last_vel, ang_p = data1.split(':')
            dst_last_pos, dst_err, dst_last_vel, dst_p = data2.split(':')
            lastY = float(ang_last_pos)
            R, P, Y = PlatformStatics.WHEEL_RPY_CONVERTER[self.__wheel_id](0, 0, lastY)
            current_distance = float(dst_last_pos)
            distance_delta = current_distance - self.__prev_distance
            self.__dx = distance_delta * math.cos(lastY)
            self.__dy = distance_delta * math.sin(lastY)
            self.__x += self.__dx
            self.__y += self.__dy
            self.__last_msg_id = int(msg_id)
            rospy.loginfo(f"Parsed wheel_{self.__wheel_id}: {raw_data}")
            return self.update(0, 0, 0, R, P, Y)
        except ValueError:
            rospy.logwarn(f"Error Parsing: {raw_data}")

class TF2Platform(TF2Link):

    def __init__(self, base_link_name='/base_link', map_name='/map', base_wheel_prefix='/base_wheel_', wheel_prefix='/wheel_'):
        TF2Link.__init__(self, base_link_name, TF2BaseLink(map_name))
        self.__lock = threading.Lock()
        self._tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.__wheels = []
        self.__platform_tf2 = []
        self.__platform_tf2_state = []
        for c in range(PlatformStatics.WHEEL_NUM):
            x, y, z = PlatformStatics.WHEELS_TRANSLATIONS_XYZ[c]
            wheel = TF2WheelWithPivot(c, self, x, y, z, base_wheel_prefix, wheel_prefix)
            self.__wheels.append(wheel)
            self.__platform_tf2.append(None)
            self.__platform_tf2_state.append(False)
        self._Y = []

    def update_Y(self, Y):
        self._Y.append(Y)
        if len(self._Y) > 10:
            self._Y.pop(0)
        rospy.loginfo(sum(self._Y))
        rospy.loginfo(len(self._Y))
        result = sum(self._Y)/float(len(self._Y))
        rospy.loginfo(result)
        return result


    def parse_serial(self, wheel_id, raw_data):
        wheel_t = self.__wheels[wheel_id].parse_wheel(raw_data)
        with self.__lock:
            if wheel_t is not None:
                self.__platform_tf2[wheel_id] = wheel_t
                self.__platform_tf2_state[wheel_id] = True
            if all(self.__platform_tf2_state):
                sum_x, sum_y, sum_z = 0, 0, 0
                xyz_s = []
                for c in range(PlatformStatics.WHEEL_NUM):
                    rospy.loginfo(f"Publishing wheel_{c} [{self.__wheels[c].link_name}] tf2")
                    self._tf_broadcaster.sendTransform(self.__platform_tf2[c])
                    self.__platform_tf2_state[c] = False
                    x, y, z = self.__wheels[c].delta_xyz
                    xyz_s.append((x, y, z))
                    sum_x += x
                    sum_y += y
                    sum_z += z
                x = sum_x / PlatformStatics.WHEEL_NUM
                y = sum_y / PlatformStatics.WHEEL_NUM
                z = sum_z / PlatformStatics.WHEEL_NUM

                fm_point = (
                    (xyz_s[0][0] + xyz_s[1][0])/2, # average of X coords between w0 and w1
                    (xyz_s[0][1] + xyz_s[1][1])/2  # average of Y coords between w0 and w1
                )
                bm_point = (
                    (xyz_s[2][0] + xyz_s[3][0])/2, # average of X coords between w2 and w3
                    (xyz_s[2][1] + xyz_s[3][1])/2  # average of Y coords between w2 and w3
                )
                Y = math.atan2(fm_point[0]-bm_point[0], abs(fm_point[1]-bm_point[1]))

                rospy.loginfo(f"Publishing platform [{self.link_name}] tf2 [{x}, {y}, {z}, 0, 0, {Y}]")
                self._tf_broadcaster.sendTransform(self.update(x, y, z, 0, 0, Y, increment=False)) # self.update_Y(Y)


class TF2PlatformPublisher(ThreadedSerialOutputHandler, TF2Platform):

    def __init__(self, data_queue, *args, **kwargs):
        ThreadedSerialOutputHandler.__init__(self, data_queue)
        TF2Platform.__init__(self, *args, **kwargs)

    def parse_serial(self, wheel_id, raw_data):
        TF2Platform.parse_serial(self, wheel_id, raw_data)