import rospy
import time
import math
import signal

from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Bool

from .lib import env2log, Latch, time_ms

class RobotPlatform():

    def __init__(self):
        rospy.init_node('robot_testing', log_level=env2log())

        goal_topic_name = rospy.get_param("~goal_topic")
        self.__goal_pub = rospy.Publisher(goal_topic_name, PoseStamped, queue_size=10)

        goal_reached_topic_name = rospy.get_param("~goal_reached")
        rospy.Subscriber(goal_reached_topic_name, Bool, self._goal_reached_callback)


        # points = rospy.get_param("~points", '[]')
        
        # self.__points = [[int(x.strip(' ')) for x in ss.lstrip(' [,').split(', ')] for ss in points.rstrip(']').split(']')]
        # for p in self.__points:
        #     print(f"")

        self.__points = [
            (0,     0   ,),
            (0.2,   0   ,),
            (0.2,   0.2 ,),
            (0,     0.2 ,)
        ]
        self.__point_id = 0

        self.__running = True
        self.__reached = False

        while self.__running and rospy.Time.now() == 0:
            rospy.logwarn(f"Client didn't receive time on /time topic yet!")
            time.sleep(1)

    def get_next_point(self):
        self.__point_id = (self.__point_id + 1)%len(self.__points)
        p = Point()
        p.x, p.y = self.__points[self.__point_id]
        return p
    
    def stop(self, *args, **kwargs):
        rospy.loginfo(f"Stopping gracefully.")
        self.__running = False

    def _goal_reached_callback(self, data):
        self.__reached = data.data

    def process(self):
        self.__running = True
        while self.__running:
            if self.__reached:
                point = self.get_next_point()
                rospy.loginfo(f"Sending new coordinates [x,y]: [{point.x},{point.y}]")
                p = PoseStamped()
                p.header.stamp = rospy.Time.now()
                p.header.frame_id = "map"
                p.pose.position = point
                p.pose.orientation.w = 1.0
                # header: 
                #   seq: 0
                #   stamp: 
                #     secs: 1657477465
                #     nsecs: 174402151
                #   frame_id: "map"
                # pose: 
                #   position: 
                #     x: 1.0230674743652344
                #     y: 0.5628037452697754
                #     z: 0.0
                #   orientation: 
                #     x: 0.0
                #     y: 0.0
                #     z: 0.12555768232363704
                #     w: 0.9920863210474765
                # ---
                self.__goal_pub.publish(p)
                time.sleep(1)
            time.sleep(0.1)

def main():
    robot = RobotPlatform()
    signal.signal(signal.SIGINT, robot.stop)
    signal.signal(signal.SIGTERM, robot.stop)
    robot.process()