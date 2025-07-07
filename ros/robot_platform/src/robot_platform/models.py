from pydantic import BaseModel, ValidationError, ConfigDict, ValidationError
from pydantic.alias_generators import to_camel

from uuid import UUID, uuid4

from typing import Optional, List, Any, Union
import json
import math
import rospy
from std_msgs.msg import Int32, String
from sensor_msgs.msg import BatteryState, NavSatFix, NavSatStatus, Imu
from geometry_msgs.msg import TransformStamped, Vector3, PoseStamped
from geometry_msgs.msg import Quaternion as ROSQuaternion

from robot_platform.msg import PlatformStatus, ServoStatus, MotorStatus, PlatformRequest, MotorRequest, ServoRequest

from .tf_helpers import get_quaterion_from_rpy, limit_angle
from .platform_statics import PlatformStatics


class Message(BaseModel):
    model_config = ConfigDict(arbitrary_types_allowed=True, populate_by_name=True)
    # model_config = ConfigDict(arbitrary_types_allowed=True, populate_by_name=True, alias_generator=to_camel)


class Battery(BaseModel):
    voltage: float

    def parse_ROS_Battery(self, header_frame_id:str, timestamp:Optional[rospy.Time] = None) -> BatteryState:
        battery = BatteryState()
        battery.header.stamp = timestamp or rospy.Time.now()
        battery.header.frame_id = header_frame_id

        battery.voltage = self.voltage
        battery.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        battery.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        battery.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        return battery

class Quaternion(BaseModel):
    w: float
    x: float
    y: float
    z: float

    @staticmethod
    def from_ROS_quaternion(q:ROSQuaternion) -> 'Quaternion':
        return Quaternion(w=q.w, x=q.x, y=q.y, z=q.z)


class Gyroscope(BaseModel):
    x: float
    y: float
    z: float

class Accelerometer(BaseModel):
    x: float
    y: float
    z: float

class IMU(BaseModel):
    quaternion: Quaternion = Quaternion.from_ROS_quaternion(get_quaterion_from_rpy(0, 0, 0))
    gyroscope: Gyroscope = Gyroscope(x=0, y=0, z=0)
    accelerometer: Accelerometer = Accelerometer(x=0, y=0, z=0)

    def to_ROS_Imu(self, header_frame_id:str, timestamp:Optional[rospy.Time] = None) -> Imu:
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
    
    def to_ROS_TF2(self, child_frame_id:str, base_frame_id:str, translation:Optional[Vector3] = None, timestamp:Optional[rospy.Time] = None) -> TransformStamped:
        imu = self.to_ROS_Imu(base_frame_id, timestamp)
        t = TransformStamped()
        t.header = imu.header
        t.header.frame_id = child_frame_id
        t.child_frame_id = base_frame_id
        if translation:
            t.transform.translation = translation
        t.transform.rotation = imu.orientation
        return t

class Servo(Message):
    angle: float

    @staticmethod
    def from_ROS_ServoStatus(status: ServoStatus) -> 'Servo':
        return Servo(angle = status.angle)
    
    @staticmethod
    def from_ROS_ServoStatus_and_delta_angle(status: ServoStatus, delta_angle:float) -> Union['Servo', None]:
        if abs(delta_angle) > PlatformStatics.MIN_ANGLE_DIFF:
            servo = Servo.from_ROS_ServoStatus(status)
            servo.angle = round(delta_angle + servo.angle, 3)
            return servo
        return None

    def __eq__(self, other:Any):
        return isinstance(other, Servo) and self.angle == other.angle
    
    def to_ROS_TF2(self, child_frame_id:str, base_frame_id:str, translation:Optional[Vector3] = None, timestamp:Optional[rospy.Time] = None) -> TransformStamped:
        t = TransformStamped()
        t.header.stamp = timestamp or rospy.Time.now()
        t.header.frame_id = child_frame_id
        t.child_frame_id = base_frame_id
        if translation:
            t.transform.translation = translation
        t.transform.rotation = get_quaterion_from_rpy(0, 0, self.angle)
        return t
    

class Motor(Message):
    velocity: float
    distance: float = 0.0

    @staticmethod
    def from_ROS_MotorStatus(status: MotorStatus) -> 'Motor':
        return Motor(velocity=status.velocity, distance=status.distance)

    def __eq__(self, other:Any):
        return isinstance(other, Motor) and self.velocity == other.velocity and self.distance == other.distance
    
    def to_ROS_TF2(self, child_frame_id:str, base_frame_id:str, wheel_diameter:float = 0, translation:Optional[Vector3] = None, timestamp:Optional[rospy.Time] = None) -> TransformStamped:
        t = TransformStamped()
        t.header.stamp = timestamp or rospy.Time.now()
        t.header.frame_id = child_frame_id
        t.child_frame_id = base_frame_id
        if translation:
            t.transform.translation = translation
        if wheel_diameter > 0:
            t.transform.rotation = get_quaterion_from_rpy(0, 0, limit_angle(self.distance / wheel_diameter))
        return t

class GPS(BaseModel):
    fix: bool
    fix_quality:float = -1
    satellites:float = 0
    speed:float = 0
    angle:float = 0
    altitude:float = -1
    dec_latitude:float = -1
    dec_longitude:float = -1

    def to_ROS_NavSatFix(self, header_frame_id:str, timestamp:Optional[rospy.Time] = None) -> NavSatFix:
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


class Status(Message):
    micros: int
    battery: Battery
    imu: IMU
    gps: GPS
    motor1: Motor
    motor2: Motor
    motor3: Motor
    motor4: Motor
    servo1: Servo
    servo2: Servo
    servo3: Servo
    servo4: Servo
    pan: Servo
    tilt: Servo
    temp: int
    move_uuid: Optional[str] = None

    @property
    def motor_list(self) -> List[Motor]:
        return [
            self.motor1, self.motor2, self.motor3, self.motor4
        ]
    @property
    def servo_list(self) -> List[Servo]:
        return [
            self.servo1, self.servo2, self.servo3, self.servo4
        ]

    def to_ROS_PlatformStatus(self, header_frame_id:str, timestamp:Optional[rospy.Time] = None) -> PlatformStatus:
        platform_status = PlatformStatus()
        platform_status.header.stamp = timestamp or rospy.Time.now()
        platform_status.header.frame_id = header_frame_id

        platform_status.imu = self.imu.to_ROS_Imu(header_frame_id, timestamp)
        platform_status.gps = self.gps.to_ROS_NavSatFix(header_frame_id, timestamp)
        platform_status.battery = self.battery.parse_ROS_Battery(header_frame_id, timestamp)

        platform_status.motor1.distance = self.motor1.distance
        platform_status.motor1.velocity = self.motor1.velocity
        platform_status.servo1.angle = self.servo1.angle
        platform_status.motor2.distance = self.motor2.distance
        platform_status.motor2.velocity = self.motor2.velocity
        platform_status.servo2.angle = self.servo2.angle
        platform_status.motor3.distance = self.motor3.distance
        platform_status.motor3.velocity = self.motor3.velocity
        platform_status.servo3.angle = self.servo3.angle
        platform_status.motor4.distance = self.motor4.distance
        platform_status.motor4.velocity = self.motor4.velocity
        platform_status.servo4.angle = self.servo4.angle
        platform_status.pan.angle = self.pan.angle
        platform_status.tilt.angle = self.tilt.angle
        platform_status.temp = Int32(data=self.temp)
        platform_status.moveUuid = String(data=self.move_uuid)
        return platform_status
    
    @staticmethod
    def from_ROS_PlatformStatus(platform_status:PlatformStatus) -> 'Status':
        status = Status(
            micros = 0,
            temp = 0,
            battery = Battery(voltage=0),
            imu = IMU(),
            gps = GPS(fix=False),
            motor1 = Motor(velocity=platform_status.motor1.velocity, distance=platform_status.motor1.distance),
            motor2 = Motor(velocity=platform_status.motor2.velocity, distance=platform_status.motor2.distance),
            motor3 = Motor(velocity=platform_status.motor3.velocity, distance=platform_status.motor3.distance),
            motor4 = Motor(velocity=platform_status.motor4.velocity, distance=platform_status.motor4.distance),
            servo1 = Servo(angle=platform_status.servo1.angle),
            servo2 = Servo(angle=platform_status.servo2.angle),
            servo3 = Servo(angle=platform_status.servo3.angle),
            servo4 = Servo(angle=platform_status.servo4.angle),
            pan = Servo(angle=platform_status.pan.angle),
            tilt = Servo(angle=platform_status.tilt.angle),
        )
        return status

def parse_response(raw_input:str) -> Optional[Status]:
    try:
        message_json = json.loads(raw_input)
        return Status.model_validate(message_json)
    except json.decoder.JSONDecodeError as _json_error:
        rospy.logerr(f"Couldn't parse JSON: '{raw_input}'")
        return None
    except ValidationError as _pydantic_error:
        rospy.logerr(f"Couldn't parse Pydantic: '{raw_input}'")
        return None
    except AttributeError as _attribute_error:
        rospy.logerr(f"Couldn't parse Pydantic: '{raw_input}'")
        return None

class Request(Message):
    duration: float = 0
    motor1: Optional[Motor] = None
    motor2: Optional[Motor] = None
    motor3: Optional[Motor] = None
    motor4: Optional[Motor] = None
    servo1: Optional[Servo] = None
    servo2: Optional[Servo] = None
    servo3: Optional[Servo] = None
    servo4: Optional[Servo] = None
    pan: Optional[Servo] = None
    tilt: Optional[Servo] = None
    move_uuid: UUID = uuid4()

    @property
    def motor_list(self) -> List[Motor]:
        return [
            self.motor1 or Motor(velocity=0, distance=0),
            self.motor2 or Motor(velocity=0, distance=0),
            self.motor3 or Motor(velocity=0, distance=0),
            self.motor4 or Motor(velocity=0, distance=0)
        ]
    @property
    def servo_list(self) -> List[Servo]:
        return [
            self.servo1 or Servo(angle=0),
            self.servo2 or Servo(angle=0),
            self.servo3 or Servo(angle=0),
            self.servo4 or Servo(angle=0)
        ]

    @staticmethod
    def from_Status(status:Status) -> 'Request':
        request = Request()
        for s, r in zip(status.motor_list, request.motor_list):
            r.velocity = s.velocity
            r.distance = s.distance
        for s, r in zip(status.servo_list, request.servo_list):
            r.angle = s.angle
        request.pan = Servo(angle=status.pan.angle)
        request.tilt = Servo(angle=status.tilt.angle)
        return request

    @staticmethod
    def from_ROS_PlatformStatus(platform_status:PlatformStatus) -> 'Request':
        return Request.from_Status(Status.from_ROS_PlatformStatus(platform_status))
    
    @staticmethod
    def from_ROS_PlatformRequest(platform_request:PlatformRequest) -> 'Request':
        request = Request()
        request.duration = platform_request.duration.data.to_sec()

        if platform_request.motor1.defined:
            request.motor1 = Motor(velocity=platform_request.motor1.velocity, distance=platform_request.motor1.distance)
        if platform_request.motor2.defined:
            request.motor2 = Motor(velocity=platform_request.motor2.velocity, distance=platform_request.motor2.distance)
        if platform_request.motor3.defined:
            request.motor3 = Motor(velocity=platform_request.motor3.velocity, distance=platform_request.motor3.distance)
        if platform_request.motor4.defined:
            request.motor4 = Motor(velocity=platform_request.motor4.velocity, distance=platform_request.motor4.distance)
        
        if platform_request.servo1.defined:
            request.servo1 = Servo(angle=platform_request.servo1.angle)
        if platform_request.servo2.defined:
            request.servo2 = Servo(angle=platform_request.servo2.angle)
        if platform_request.servo3.defined:
            request.servo3 = Servo(angle=platform_request.servo3.angle)
        if platform_request.servo4.defined:
            request.servo4 = Servo(angle=platform_request.servo4.angle)
    
        if platform_request.pan.defined:
            request.pan = Servo(angle=platform_request.pan.angle)
        if platform_request.tilt.defined:
            request.tilt = Servo(angle=platform_request.tilt.angle)
        
        return request
    model_config = ConfigDict(arbitrary_types_allowed=True, populate_by_name=True, alias_generator=to_camel)
