import rospy
import time
import math
import signal

from geometry_msgs.msg import PoseStamped, Pose2D, TransformStamped, Point
from tf2_ros import TransformBroadcaster
import tf_conversions


from .lib import env2log, ROBOT_WIDTH_M, Rate, SupressedLog

class RobotPlatform():

    def __init__(self):
        rospy.init_node('robot', log_level=env2log())

        goal_topic_name = rospy.get_param("~goal_topic")
        input_topic_name = rospy.get_param("~input_topic")
        output_topic_name = rospy.get_param("~output_topic")
        self._tf2_link_base = rospy.get_param("~tf2_base_link")
        self._tf2_link_child = rospy.get_param("~tf2_map_link")

        rospy.Subscriber(goal_topic_name, PoseStamped, self._goal_callback)
        rospy.Subscriber(input_topic_name, Pose2D, self._pose2d_callback)
        self.__output_pub = rospy.Publisher(output_topic_name, Point, queue_size=10)
        self.__tf_broadcaster = TransformBroadcaster()

        self.__tf2_suppressed_log = SupressedLog(1)
        self.__tf2_rate = Rate()
        
        self.__pose2d = Pose2D()
        self.__running = True
        while self.__running and rospy.Time.now() == 0:
            rospy.logwarn(f"Client didn't receive time on /time topic yet!")
            time.sleep(1)

    def _pose2d_callback(self, data):
        platform_transform = TransformStamped()
        platform_transform.header.stamp = rospy.Time.now()
        platform_transform.header.frame_id = self._tf2_link_child
        platform_transform.child_frame_id = self._tf2_link_base
        platform_transform.transform.translation.x = data.x
        platform_transform.transform.translation.y = data.y
        platform_transform.transform.translation.z = 0.0
        q = tf_conversions.transformations.quaternion_from_euler(0.0, 0.0, data.theta)
        platform_transform.transform.rotation.x = q[0]
        platform_transform.transform.rotation.y = q[1]
        platform_transform.transform.rotation.z = q[2]
        platform_transform.transform.rotation.w = q[3]
        self.__tf_broadcaster.sendTransform(platform_transform)
        self.__pose2d = data
        self.__tf2_rate.update()
        self.__tf2_suppressed_log.handler(rospy.loginfo, f"TF2 update rate {self.__tf2_rate.mean_rate}ms")
    
    def stop(self, *args, **kwargs):
        rospy.loginfo(f"Stopping gracefully.")
        self.__running = False

    def _goal_callback(self, data):
        rospy.loginfo(f"Started processing goal {data.pose.position.x} {data.pose.position.y} {data.pose.position.z}")

        dx = data.pose.position.x - self.__pose2d.x
        dy = data.pose.position.y - self.__pose2d.y

        r, p, y = tf_conversions.transformations.euler_from_quaternion([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])
        
        angle = math.atan2(dy, dx) - self.__pose2d.theta
        if angle > math.pi:
            angle = math.pi - angle
        if angle < -math.pi:
            angle = -math.pi + angle
        
        distance = math.sqrt(dx*dx + dy*dy)

        rospy.loginfo(f"Rotating {angle} and moving {distance}")

        turn_distance = (angle * ROBOT_WIDTH_M /2) * 100.0
        turn_time = abs(turn_distance/2)
        turn_point = Point()
        turn_point.x, turn_point.y, turn_point.z = -turn_distance, turn_distance, turn_time
        self.__output_pub.publish(turn_point)

        move_time = 100.0 * distance / 2
        move_point = Point()
        move_point.x, move_point.y, move_point.z = distance*100.0, distance*100.0, move_time
        self.__output_pub.publish(move_point)

    def process(self):
        self.__running = True
        while self.__running:
            time.sleep(1)

def main():
    robot = RobotPlatform()
    signal.signal(signal.SIGINT, robot.stop)
    signal.signal(signal.SIGTERM, robot.stop)
    robot.process()