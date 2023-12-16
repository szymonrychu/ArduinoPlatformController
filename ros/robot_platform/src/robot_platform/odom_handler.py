
from .message_utils import MotorStatus

from geometry_msgs.msg import Pose

class OdomHanlder():

    def handle_motor_updates(self, motor1:MotorStatus, motor2:MotorStatus, motor3:MotorStatus, motor4:MotorStatus):
        pass
    
    def getPlatformPose() -> Pose:
        p = Pose()
        return p