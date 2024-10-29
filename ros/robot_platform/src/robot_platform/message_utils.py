from pydantic import BaseModel, Field, validator, ValidationError, ConfigDict, ValidationError
from typing import Optional, List
from uuid import uuid4, UUID
import json
import rospy
from sensor_msgs.msg import BatteryState, NavSatFix, NavSatStatus, Imu
from geometry_msgs.msg import TransformStamped, Vector3, PoseStamped
from robot_platform.msg import PlatformStatus, MoveRequest, Motor, Servo

from .tf_helpers import get_quaterion_from_rpy

class Message(BaseModel):
    model_config = ConfigDict(arbitrary_types_allowed=True, populate_by_name=True)

class BatteryStatus(BaseModel):
    voltage: float

    def parse_ROS_Battery(self, header_frame_id:str, timestamp:rospy.Time=None) -> BatteryState:
        battery = BatteryState()
        battery.header.stamp = timestamp or rospy.Time.now()
        battery.header.frame_id = header_frame_id

        battery.voltage = self.voltage
        battery.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        battery.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        battery.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        return battery

class QuaternionStatus(BaseModel):
    w: float
    x: float
    y: float
    z: float

class GyroscopeStatus(BaseModel):
    x: float
    y: float
    z: float

class AccelerometerStatus(BaseModel):
    x: float
    y: float
    z: float

class IMUStatus(BaseModel):
    quaternion: QuaternionStatus
    gyroscope: GyroscopeStatus
    accelerometer: AccelerometerStatus

    def parse_ROS_IMU(self, header_frame_id:str, timestamp:rospy.Time=None) -> Imu:
        imu = Imu()
        imu.header.stamp = timestamp or rospy.Time.now()
        imu.header.frame_id = header_frame_id

        imu.orientation.w = self.quaternion.w
        imu.orientation.x = self.quaternion.x
        imu.orientation.y = self.quaternion.y
        imu.orientation.z = self.quaternion.z
        imu.orientation_covariance = (
            0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01
        )


        imu.angular_velocity.x = self.gyroscope.x
        imu.angular_velocity.y = self.gyroscope.y
        imu.angular_velocity.z = self.gyroscope.z
        imu.angular_velocity_covariance = (
            0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01
        )

        imu.linear_acceleration.x = self.accelerometer.x
        imu.linear_acceleration.y = self.accelerometer.y
        imu.linear_acceleration.z = self.accelerometer.z
        imu.linear_acceleration_covariance = (
            0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01
        )
        return imu
    
    def parse_ROS_TF(self, child_frame_id:str, base_frame_id:str, translation:Vector3 = None, timestamp:rospy.Time=None) -> TransformStamped:
        imu = self.parse_ROS_IMU(base_frame_id, timestamp)
        t = TransformStamped()
        t.header = imu.header
        t.header.frame_id = child_frame_id
        t.child_frame_id = base_frame_id
        if translation:
            t.transform.translation = translation
        t.transform.rotation = imu.orientation
        return t

class ServoStatus(Message):
    angle: Optional[float] = None
    angle_provided: Optional[bool] = False

    def __eq__(self, other):
        return self.angle == other.angle and self.angle_provided == other.angle_provided
    
    def parse_ROS_TF(self, child_frame_id:str, base_frame_id:str, translation:Vector3 = None, timestamp:rospy.Time=None) -> TransformStamped:
        t = TransformStamped()
        t.header.stamp = timestamp or rospy.Time.now()
        t.header.frame_id = child_frame_id
        t.child_frame_id = base_frame_id
        if translation:
            t.transform.translation = translation
        t.transform.rotation = get_quaterion_from_rpy(0, 0, self.angle)
        return t

class MotorStatus(ServoStatus):
    velocity: Optional[float] = None
    distance: Optional[float] = None
    angle: Optional[float] = None
    angle_provided: Optional[bool] = False

    def __eq__(self, other):
        return self.velocity == other.velocity and self.distance == other.distance and self.angle == other.angle and self.angle_provided == other.angle_provided

class GPSStatus(BaseModel):
    fix: bool
    fix_quality: Optional[int] = -1
    satellites: Optional[int] = 0
    speed: Optional[float] = 0
    angle: Optional[float] = 0
    altitude: Optional[float] = -1
    dec_latitude: Optional[float] = -1
    dec_longitude: Optional[float] = -1

    def parse_ROS_GPS(self, header_frame_id:str, timestamp:rospy.Time=None) -> NavSatFix:
        nav_sat_status = NavSatStatus()
        nav_sat_fix = NavSatFix()
        nav_sat_fix.header.stamp = timestamp or rospy.Time.now()
        nav_sat_fix.header.frame_id = header_frame_id
        if self.fix:
            nav_sat_status.status = NavSatStatus.STATUS_FIX
            nav_sat_status.service = NavSatStatus.SERVICE_GPS
            nav_sat_fix.altitude = self.altitude
            nav_sat_fix.latitude = self.dec_latitude
            nav_sat_fix.longitude = self.dec_longitude
            nav_sat_fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        else:
            nav_sat_status.status = NavSatStatus.STATUS_NO_FIX
        nav_sat_fix.status = nav_sat_status
        return nav_sat_fix
    
class MotorServoStatuses(Message):
    motor1_servo: ServoStatus
    motor2_servo: ServoStatus
    motor3_servo: ServoStatus
    motor4_servo: ServoStatus

    def __eq__(self, other):
        return self.motor1_servo == other.motor1_servo and self.motor2_servo == other.motor2_servo and \
            self.motor3_servo == other.motor3_servo and self.motor4_servo == other.motor4_servo


class StatusResponse(Message):
    micros: int
    int_temp: int
    move_duration: float
    battery: BatteryStatus
    imu: IMUStatus
    gps: GPSStatus
    motor1: MotorStatus
    motor2: MotorStatus
    motor3: MotorStatus
    motor4: MotorStatus
    pan: ServoStatus
    tilt: ServoStatus

    @property
    def motor_list(self) -> List[MotorStatus]:
        return [
            self.motor1, self.motor2, self.motor3, self.motor4
        ]

    def parse_ROS(self, header_frame_id:str, timestamp:rospy.Time=None) -> PlatformStatus:
        platform_status = PlatformStatus()
        platform_status.header.stamp = timestamp or rospy.Time.now()
        platform_status.header.frame_id = header_frame_id

        platform_status.motor1.distance = self.motor1.distance
        platform_status.motor1.velocity = self.motor1.velocity
        platform_status.motor1.servo.angle = self.motor1.angle
        platform_status.motor2.distance = self.motor2.distance
        platform_status.motor2.velocity = self.motor2.velocity
        platform_status.motor2.servo.angle = self.motor2.angle
        platform_status.motor3.distance = self.motor3.distance
        platform_status.motor3.velocity = self.motor3.velocity
        platform_status.motor3.servo.angle = self.motor3.angle
        platform_status.motor4.distance = self.motor4.distance
        platform_status.motor4.velocity = self.motor4.velocity
        platform_status.motor4.servo.angle = self.motor4.angle
        platform_status.pan.angle = self.pan.angle
        platform_status.tilt.angle = self.tilt.angle
        platform_status.imu = self.imu.parse_ROS_IMU(header_frame_id, timestamp)
        platform_status.gps = self.gps.parse_ROS_GPS(header_frame_id, timestamp)
        platform_status.battery = self.battery.parse_ROS_Battery(header_frame_id, timestamp)
        return platform_status

class Request(Message):
    motor1: Optional[MotorStatus] = MotorStatus()
    motor2: Optional[MotorStatus] = MotorStatus()
    motor3: Optional[MotorStatus] = MotorStatus()
    motor4: Optional[MotorStatus] = MotorStatus()
    pan: Optional[ServoStatus] = ServoStatus()
    tilt: Optional[ServoStatus] = ServoStatus()
    move_duration: Optional[float] = 0.0

    def __eq__(self, other):
        return self.move_duration == other.move_duration and \
            self.motor1 == other.motor1 and \
            self.motor2 == other.motor2 and \
            self.motor3 == other.motor3 and \
            self.motor4 == other.motor4 and \
            self.pan == other.pan and \
            self.tilt == other.tilt

    @staticmethod
    def from_MoveRequest(m:MoveRequest):
        r = Request()
        r.move_duration = round(m.duration, 3)

        r.motor1 = MotorStatus()
        r.motor1.velocity = round(m.motor1.velocity, 3)
        if m.motor1.servo.angle_provided:
            r.motor1.angle = round(m.motor1.servo.angle, 5)
            
        r.motor2 = Motor()
        r.motor2.velocity = round(m.motor2.velocity, 3)
        if m.motor2.servo.angle_provided:
            r.motor2.angle = round(m.motor2.servo.angle, 5)

        r.motor3 = MotorStatus()
        r.motor3.velocity = round(m.motor3.velocity, 3)
        if m.motor3.servo.angle_provided:
            r.motor3.angle = round(m.motor3.servo.angle, 5)

        r.motor4 = MotorStatus()
        r.motor4.velocity = round(m.motor4.velocity, 3)
        if m.motor4.servo.angle_provided:
            r.motor4.angle = round(m.motor4.servo.angle, 5)
        
        r.pan = ServoStatus()
        if m.pan.angle and m.pan.angle_provided:
            r.pan.angle = round(m.pan.angle, 5)

        r.tilt = ServoStatus()
        if m.tilt.angle and m.tilt.angle_provided:
            r.tilt.angle = round(m.tilt.angle, 5)
        
        return r
    



def parse_response(raw_input:str) -> StatusResponse:
    try:
        message_json = json.loads(raw_input)
        return StatusResponse.model_validate(message_json)
    except json.decoder.JSONDecodeError as _json_error:
        rospy.logerr(f"Couldn't parse JSON: '{raw_input}'")
        return None
    except ValidationError as _pydantic_error:
        rospy.logerr(f"Couldn't parse Pydantic: '{raw_input}'")
        return None
    except AttributeError as _attribute_error:
        rospy.logerr(f"Couldn't parse Pydantic: '{raw_input}'")
        return None

