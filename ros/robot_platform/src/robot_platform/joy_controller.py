
import math
import signal

from .odometry_helpers import PlatformStatics, create_request
import rospy
from .ros_helpers import ROSNode

from sensor_msgs.msg import Joy, JoyFeedback
from robot_platform.msg import PlatformStatus, MoveRequest
from geometry_msgs.msg import Point

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

class JoyPlatformController(ROSNode):

    def __init__(self):
        ROSNode.__init__(self)
        self._last_platform_status = PlatformStatus()
        self._last_joy = Joy()
        self._last_limited_deltas = [0.0] * PlatformStatics.MOTOR_NUM
        self._last_request = None
        self._autorepeat_rate = rospy.get_param('~autorepeat_rate')
        joy_input_topic = rospy.get_param('~joy_topic')
        joy_output_topic = rospy.get_param('~joy_feedback_topic')
        move_request_output_topic = rospy.get_param('~move_request_output_topic')
        platform_status_input_topic = rospy.get_param('~platform_status_input_topic')

        rospy.Subscriber(joy_input_topic, Joy, self._handle_joystick_updates)
        self._joy_feedback_publisher = rospy.Publisher(joy_output_topic, JoyFeedback)
        self._move_request_publisher = rospy.Publisher(move_request_output_topic, MoveRequest)
        rospy.Subscriber(platform_status_input_topic, PlatformStatus, self._handle_platform_status)

        rospy.Timer(rospy.Duration(duration), self._send_request)
        self.spin()

    def _handle_joystick_updates(self, data:Joy):
        self._last_joy = data

    def _handle_platform_status(self, status:PlatformStatus):
        self._last_platform_status = status

    def _send_request(self, event=None):
        if self._last_joy.axes:
            rel_velocity = -0.25 * self._last_joy.axes[1]
            if rel_velocity < 0:
                rel_velocity = rel_velocity
            elif rel_velocity > 0:
                rel_velocity = rel_velocity
            # rel_velocity = 0.3
            
            turn_radius = round(-0.95 * self._last_joy.axes[0], 2)
            if turn_radius < 0:
                turn_radius = max(turn_radius, -0.99)
            elif turn_radius > 0:
                turn_radius = min(turn_radius, 0.99)
            if turn_radius > 0.01:
                turn_radius = 1 - turn_radius
            elif turn_radius < -0.01:
                turn_radius = -1 - turn_radius
            # turn_radius = -0.2


            velocity = 0.0
            boost = 1.0 + 3*(-self._last_joy.axes[3]+1)/2
            if abs(rel_velocity) > 0.01:
                velocity = round(-PlatformStatics.MOVE_VELOCITY * (rel_velocity * boost), 2)
            
            turning_point = None
            if abs(turn_radius) > 0.0001:
                turning_point = Point()
                turning_point.x = turn_radius

            r = create_request(velocity, duration, self._last_platform_status, turning_point)
            if self._last_request != r:
                self._move_request_publisher.publish(r)
            self._last_request = r




def main():
    platform = JoyPlatformController()
    signal.signal(signal.SIGINT, platform.stop)
    signal.signal(signal.SIGTERM, platform.stop)
    platform.start()