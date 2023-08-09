#!/usr/bin/env python3

import rospy
import sys

from std_msgs.msg import String


class FireForgetPublisher():
    def __init__(self, topic_n, data):
        self._data = String()
        self._data.data = data
        self._publisher = rospy.Publisher(topic_n, String, queue_size=10)

    def publish(self, *_args, **_kwargs):
        self._publisher.publish(self._data)
        rospy.signal_shutdown('')

    def start(self):
        rospy.init_node('tmp_node')
        rospy.Timer(rospy.Duration(0.1), callback=self.publish, oneshot=True)
        rospy.spin()

FireForgetPublisher(sys.argv[1], sys.argv[2]).start()