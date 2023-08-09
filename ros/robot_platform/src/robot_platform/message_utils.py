from pydantic import BaseModel, Field, validator, ValidationError, ConfigDict
from typing import Optional, List
import json

class Response(BaseModel):
    model_config = ConfigDict(arbitrary_types_allowed=True, populate_by_name=True)

class BatteryStatus(BaseModel):
    voltage: float

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

class MotorStatus(BaseModel):
    ready: bool
    servo: float
    distance: float
    distance_error: float
    distance_steering: float
    velocity: float
    velocity_error: float
    velocity_steering: float
    steering: float

class GPSStatus(BaseModel):
    fix_quality: int
    satellites: int
    speed: float
    angle: float
    altitude: float
    dec_latitude: float
    dec_longitude: float

class StatusResponse(Response):
    micros: int
    message_type: str
    status: str
    queue_l: int
    int_temp: int
    battery: BatteryStatus
    imu: IMUStatus
    gps: Optional[GPSStatus] = None
    motor1: Optional[MotorStatus] = None
    motor2: Optional[MotorStatus] = None
    motor3: Optional[MotorStatus] = None
    motor4: Optional[MotorStatus] = None

    @validator('message_type')
    def type_is_success_or_error(cls, v):
        if v != 'STATUS':
            raise ValidationError('Message type not eq STATUS')
        return v

class AckResponse(Response):
    micros: int
    message_type: str
    message: str

    @validator('message_type')
    def type_is_success_or_error(cls, v):
        if v not in ['ERROR', 'SUCCESS']:
            raise ValidationError('Message type not in [ERROR, SUCCCESS]')
        return v




class Request(BaseModel):
    model_config = ConfigDict(arbitrary_types_allowed=True, populate_by_name=True)

class MotorRequest(BaseModel):
    distance: float
    velocity: float
    angle: float

class RawRequest(Request):
    message_type: str = Field(default='raw_move')
    motor1: Optional[MotorRequest]
    motor2: Optional[MotorRequest]
    motor3: Optional[MotorRequest]
    motor4: Optional[MotorRequest]

class ResetRequest(Request):
    message_type: str = Field(default='reset')

class ResetQueueRequest(Request):
    message_type: str = Field(default='reset_queue')

class StopRequest(Request):
    message_type: str = Field(default='stop')

class MoveRequest(Request):
    pass

class TurnRequest(MoveRequest):
    message_type: str = Field(default='turn')
    turn_angle: float
    turn_velocity: float
    turn_x: float = Field(default=0)
    turn_y: float = Field(default=0)

class ForwardRequest(MoveRequest):
    message_type: str = Field(default='forward')
    move_distance: float
    move_velocity: float
    move_angle: float = Field(default=0)

class SequentionalMove(Request):
    message_type: str = Field(default='sequentional_move')
    moves: List[MoveRequest]



def parse_response(raw_input:str) -> Response:
    try:
        message_json = json.loads(raw_input)
        message_type = message_json["message_type"]

        if message_type == 'STATUS':
            return StatusResponse.parse_obj(message_json)
        if message_type in ['ERROR', 'SUCCESS']:
            return AckResponse.parse_obj(message_json)
    except json.decoder.JSONDecodeError as _json_error:
        return None
    
def encode_request(req:Request) -> str:
    return req.json()
