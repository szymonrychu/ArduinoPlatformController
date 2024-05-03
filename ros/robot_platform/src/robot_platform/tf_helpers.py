from typing import Tuple
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, TransformStamped, Point
import tf_conversions
import numpy as np
import rospy
import math

def add_points(p1:Point, p2:Point=None) -> Point:
    if not p2:
        return p1
    p = Point()
    p.x = p1.x + p2.x
    p.y = p1.y + p2.y
    p.z = p1.z + p2.z
    return p

def substract_points(p1:Point, p2:Point=None) -> Point:
    if not p2:
        return p1
    p = Point()
    p.x = p1.x - p2.x
    p.y = p1.y - p2.y
    p.z = p1.z - p2.z
    return p
    

def create_static_transform(root_frame_id:str, child_frame_id:str, X:float=0, Y:float=0, Z:float=0, Roll:float=0, Pitch:float=0, Yaw:float=0, timestamp:rospy.Time=None) -> TransformStamped:
    t = TransformStamped()
    t.header.stamp = timestamp or rospy.Time.now()
    t.header.frame_id = root_frame_id
    t.child_frame_id = child_frame_id
    t.transform.translation.x = X
    t.transform.translation.y = Y
    t.transform.translation.z = Z
    t.transform.rotation = get_quaterion_from_rpy(Roll, Pitch, Yaw)
    return t

def get_rpy_from_quaternion(q:Quaternion) -> Tuple[float, float, float]:
    return tf_conversions.transformations.euler_from_quaternion(
        [q.w, q.x, q.y, q.z]
    )

def get_quaterion_from_rpy(roll:float, pitch:float, yaw:float) -> Quaternion:
    q = tf_conversions.transformations.quaternion_from_euler(roll, pitch, yaw)
    return Quaternion(*q)

def normalize_angle(angle:float):
    while angle > 2 * math.pi:
        angle -= 2 * math.pi
    while angle < 2* math.pi:
        angle += 2 * math.pi
    return angle

def stabilise_quaternion(q:Quaternion) -> Quaternion:
    roll, pitch, yaw = get_rpy_from_quaternion(q)
    return get_quaterion_from_rpy(0, 0, yaw)

def pose_to_transform_stamped(pose:Pose, frame_id:str, child_frame_id:str, translation:Point=None, timestamp:rospy.Time=None) -> TransformStamped:
    t = TransformStamped()
    t.header.frame_id = frame_id
    t.header.stamp = timestamp or rospy.Time.now()
    t.child_frame_id = child_frame_id
    t.transform.translation = add_points(pose.position, translation)
    t.transform.rotation = pose.orientation
    return t

def pose_to_transform_stabilised_stamped(pose:Pose, frame_id:str, child_frame_id:str, translation:Point=None, timestamp:rospy.Time=None) -> TransformStamped:
    t = TransformStamped()
    t.header.frame_id = frame_id
    t.header.stamp = timestamp or rospy.Time.now()
    t.child_frame_id = child_frame_id
    t.transform.translation = add_points(pose.position, translation)
    t.transform.rotation = stabilise_quaternion(pose.orientation)
    return t

def pose_to_pose_stamped(pose:Pose, frame_id:StopAsyncIteration, timestamp:rospy.Time=None) -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id = frame_id
    ps.header.stamp = timestamp or rospy.Time.now()
    ps.pose.position = pose.position
    ps.pose.orientation = pose.orientation
    return ps

def difference_between_Poses(p1:Pose, p2:Pose):
    result = Pose()
    q1_inv = [
        p1.orientation.x,
        p1.orientation.y,
        p1.orientation.z,
        -p1.orientation.w,
    ]
    q2 = [
        p2.orientation.x,
        p2.orientation.y,
        p2.orientation.z,
        p2.orientation.w,
    ]

    relative_q = tf_conversions.transformations.quaternion_multiply(q2, q1_inv)
    result.orientation.x = relative_q[0]
    result.orientation.y = relative_q[1]
    result.orientation.z = relative_q[2]
    result.orientation.w = relative_q[3]


    p1_mat = np.matrix([
        p1.position.x, p1.position.y, p1.position.z
    ])
    p2_mat = np.matrix([
        [p2.position.x], [p2.position.y], [p2.position.z]
    ])

    relative_p = p2_mat - p1_mat

    result.position.x = relative_p[0][0]
    result.position.y = relative_p[1][0]
    result.position.z = relative_p[2][0]

    return result