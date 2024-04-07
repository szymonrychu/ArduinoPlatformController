from pydantic import BaseModel, Field, validator, ValidationError, ConfigDict, ValidationError
from typing import Optional, List
from uuid import uuid4, UUID
import json
import rospy
from sensor_msgs.msg import BatteryState, NavSatFix, NavSatStatus, Imu
from geometry_msgs.msg import TransformStamped, Vector3, PoseStamped
from robot_platform.msg import PlatformStatus

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

class ServoStatus(BaseModel):
    angle: Optional[float] = None
    
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

class GPSStatus(BaseModel):
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
        if self.satellites > 0:
            nav_sat_status.status = NavSatStatus.STATUS_FIX
            nav_sat_status.service = NavSatStatus.SERVICE_GPS
            nav_sat_fix.altitude = self.altitude
            nav_sat_fix.latitude = self.dec_latitude
            nav_sat_fix.longitude = self.dec_longitude
            nav_sat_fix.position_covariance = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        else:
            nav_sat_status.status = NavSatStatus.STATUS_NO_FIX
        nav_sat_fix.status = nav_sat_status
        return nav_sat_fix
    
class MotorServoStatuses(Message):
    motor1_servo: ServoStatus
    motor2_servo: ServoStatus
    motor3_servo: ServoStatus
    motor4_servo: ServoStatus


class StatusResponse(Message):
    micros: int
    int_temp: int
    move_duration: float
    battery: BatteryStatus
    imu: IMUStatus
    gps: Optional[GPSStatus] = None
    motor1: MotorStatus
    motor2: MotorStatus
    motor3: MotorStatus
    motor4: MotorStatus
    pan: ServoStatus
    tilt: ServoStatus

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
        if self.gps:
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
    
def encode_request(req:Request) -> str:
    return req.model_dump_json()
