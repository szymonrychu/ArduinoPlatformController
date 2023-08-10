from typing import Tuple
from geometry_msgs.msg import Pose
import tf_conversions
import numpy as np

def get_RPY_from_Pose(p:Pose) -> Tuple[float, float, float]:
    return tf_conversions.transformations.euler_from_quaternion(
        p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z
    )

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

    result.position.x = relative_p[0]
    result.position.y = relative_p[1]
    result.position.z = relative_p[2]

    return result