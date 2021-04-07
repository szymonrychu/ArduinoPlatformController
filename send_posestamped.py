#!/usr/bin/env python3
import rospy
import tf_conversions
from geometry_msgs.msg import PoseStamped

MOVES = [
    [(0.2, 0.0, 0.0), (0.0, 0.0, 0.0)], # go to 20cm forward
    [(0.0, 0.0, 0.0), (0.0, 0.0, 0.0)]  # go to origin
]

pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

move_num = 0
while True:
    (x, y, z), (r, p, y) = MOVES[move_num]

    goalMsg = PoseStamped()
    goalMsg.header.frame_id = '/map' # map frame
    goalMsg.pose.position.x = x
    goalMsg.pose.position.y = y
    goalMsg.pose.position.z = z

    q = tf_conversions.transformations.quaternion_from_euler(r, p y)

    goalMsg.pose.orientation = q

    pub.publish(goalMsg) 
