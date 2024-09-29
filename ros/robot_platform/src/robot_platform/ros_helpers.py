import rospy
from .log_utils import env2log
from threading import Thread

class ROSNode():

    def __init__(self, name='robot_platform'):
        rospy.init_node(name, log_level=env2log())

    def spin(self):
        rospy.spin()

    def is_running(self):
        return not rospy.is_shutdown()

    def stop(self, reason='', *_args, **_kwargs):
        rospy.signal_shutdown(reason)

class ThreadedROSNode(Thread):

    def __init__(self, name='robot_platform'):
        Thread.__init__(self, name=name, target=self.__spin)
        rospy.init_node(name, log_level=env2log())

    def start(self):
        Thread.start()

    def __spin(self):
        rospy.spin()

    def join(self, reason='', *_args, **_kwargs):
        rospy.signal_shutdown(reason)
        Thread.join(self)
