
import math
import signal

from .tf_helpers import *
from .odometry_helpers import *
import rospy
from .ros_helpers import ROSNode

from robot_platform.msg import PlatformStatus, MoveRequest
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry

'''

Index	Button
0	A (CROSS)
1	B (CIRCLE)
2	X (SQUARE)
3	Y (TRIANGLE)
4	BACK (SELECT)
5	GUIDE (Middle/Manufacturer Button)
6	START
7	LEFTSTICK
8	RIGHTSTICK
9	LEFTSHOULDER
10	RIGHTSHOULDER
11	DPAD_UP
12	DPAD_DOWN
13	DPAD_LEFT
14	DPAD_RIGHT
15	MISC1 (Depends on the controller manufacturer, but is usually at a similar location on the controller as back/start)
16	PADDLE1 (Upper left, facing the back of the controller if present)
17	PADDLE2 (Upper right, facing the back of the controller if present)
18	PADDLE3 (Lower left, facing the back of the controller if present)
19	PADDLE4 (Lower right, facing the back of the controller if present)
20	TOUCHPAD (If present. Button status only)
Index	Axis
0	LEFTX
1	LEFTY
2	RIGHTX
3	RIGHTY
4	TRIGGERLEFT
5	TRIGGERRIGHT
'''

duration = 0.1

class PathPlatformController(ROSNode):

    def __init__(self):
        ROSNode.__init__(self)
        self._last_platform_status = PlatformStatus()

        self._last_odometry = Odometry()

        self._last_pose_array = PoseArray()
        self._pose_counter = 1
        self._last_angle = 0.0

        move_request_output_topic = rospy.get_param('~move_request_output_topic')
        platform_status_input_topic = rospy.get_param('~platform_status_input_topic')
        trajectory_poses_input_topic = rospy.get_param('~trajectory_poses_input_topic')
        odometry_input_topic = rospy.get_param('~odometry_input_topic')
        
        self._move_request_publisher = rospy.Publisher(move_request_output_topic, MoveRequest)
        rospy.Subscriber(platform_status_input_topic, PlatformStatus, self._handle_platform_status)
        rospy.Subscriber(trajectory_poses_input_topic, PoseArray, self._handle_trajectory_update)
        rospy.Subscriber(odometry_input_topic, Odometry, self._handle_odometry_update)

        rospy.Timer(rospy.Duration(duration), self._send_request)
        self.spin()

    def _handle_odometry_update(self, odometry:Odometry):
        self._last_odometry = odometry

    def _handle_platform_status(self, status:PlatformStatus):
        self._last_platform_status = status

    def _handle_trajectory_update(self, poses:PoseArray):
        self._last_pose_array = poses
        self._pose_counter = 1

    def __compute_turning_point(self, angle_delta:float) -> Optional[float]:
        turning_point = None
        if angle_delta > math.pi/12:
            turning_point = Point()
            turning_point.y = (0.3 + round(min(1.0 - 10.0*angle_delta, 1.0), 1))
        if angle_delta < math.pi/12:
            turning_point = Point()
            turning_point.y = -(0.3 + round(min(1.0 - 10.0*angle_delta, 1.0), 1))
        return turning_point

    def _send_request(self, event=None):
        next_pose_to_reach = None
        try:
            next_pose_to_reach = self._last_pose_array.poses[self._pose_counter]
            self._pose_counter += 1
        except IndexError:
            return
        
        X, Y = self._last_odometry.pose.pose.position.x, self._last_odometry.pose.pose.position.y
        X_a, Y_a = next_pose_to_reach.position.x, next_pose_to_reach.position.y
        dX, dY = X_a - X, Y_a - Y
        
        alfa = math.atan2(dY, dX)
        move_distance = math.sqrt(dX**2 + dY**2)
        if move_distance < 0.1:
            self._pose_counter += 1
            return
        
        move_duration = duration
        roll, pitch, yaw = get_rpy_from_quaternion(self._last_odometry.pose.pose.orientation)
        roll_a, pitch_a, yaw_a = get_rpy_from_quaternion(next_pose_to_reach.orientation)
        
        move_velocity = 0.01 * move_distance/move_duration
        angle_delta = abs(yaw - alfa)
        rospy.loginfo(f"Angles: {rad2deg([yaw])}, {rad2deg([alfa])}, {rad2deg([angle_delta])}, distance,duration: {move_distance},{move_duration}")

        r = None
        if angle_delta < math.pi:
            rospy.loginfo(f"Going forward")
            if angle_delta < math.pi/12: # 15deg
                r = create_request(move_velocity, move_duration, self._last_platform_status, None)
            else:
                r = create_request(move_velocity, move_duration, self._last_platform_status, self.__compute_turning_point(yaw - alfa))
            

        else:
            rospy.loginfo(f"Going backward")
            if angle_delta < math.pi/12: # 15deg
                r = create_request(-move_velocity, move_duration, self._last_platform_status, None)
            else:
                r = create_request(-move_velocity, move_duration, self._last_platform_status, self.__compute_turning_point(yaw - alfa))



        if r:
            self._move_request_publisher.publish(r)

def main():
    platform = PathPlatformController()
    signal.signal(signal.SIGINT, platform.stop)
    signal.signal(signal.SIGTERM, platform.stop)
    platform.start()