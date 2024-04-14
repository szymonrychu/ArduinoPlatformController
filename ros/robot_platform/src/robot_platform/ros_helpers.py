import rospy
from .log_utils import env2log

class ROSNode():

    def __init__(self, name='robot_platform'):
        rospy.init_node(name, log_level=env2log())

    def start(self):
        rospy.spin()

    def is_running(self):
        return not rospy.is_shutdown()

    def stop(self, reason='', *_args, **_kwargs):
        rospy.signal_shutdown(reason)