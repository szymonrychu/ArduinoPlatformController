#include <Arduino.h>
#include <SimpleFOC.h>

BLDCMotor motor = BLDCMotor(7);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);
LowsideCurrentSense currentSense = LowsideCurrentSense(0.003f, -64.0f / 7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

MagneticSensorI2C sensor = MagneticSensorI2C(0x36, 12, 0x0E, 4);


Commander command = Commander(Serial);
void doTarget(char* cmd) { command.target(&motor, cmd, " "); }

void setup() {
    sensor.init();

    motor.linkSensor(&sensor);

    driver.voltage_power_supply = 12;
    driver.init();
    motor.linkDriver(&driver);

    currentSense.linkDriver(&driver);
    currentSense.init();
    currentSense.skip_align = true;
    motor.linkCurrentSense(&currentSense);

    motor.voltage_sensor_align = 1;
    motor.velocity_index_search = 3;
    motor.voltage_limit = 6;
    motor.current_limit = 10; // Amps -  default 2 Amps
    motor.velocity_limit = 0;

    // motor.controller = MotionControlType::velocity;
    motor.controller = MotionControlType::angle;
    motor.torque_controller = TorqueControlType::foc_current;
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

    // motor.PID_current_q.P = motor.PID_current_d.P = 0.3;
    // // motor.PID_current_q.I = motor.PID_current_d.D = 0.1;
    // motor.PID_current_q.I = motor.PID_current_d.I = 0.5;

    motor.PID_current_q.P = 0.1;
    motor.PID_current_q.I= 4;
    motor.LPF_current_q.Tf = 0.01; 

    motor.PID_current_d.P= 0.1;
    motor.PID_current_d.I = 4;
    motor.LPF_current_d.Tf = 0.01; 


    motor.PID_velocity.P = 0.5;
    // motor.PID_velocity.D = 0.5;
    motor.PID_velocity.I = 5;
    motor.PID_velocity.output_ramp = 1000;
    motor.LPF_velocity.Tf = 0.01;



    motor.P_angle.P = 20;
    motor.PID_velocity.I = 5;


    Serial.begin(115200);


    motor.useMonitoring(Serial);

    motor.init();
    motor.initFOC();
    command.add('T', doTarget, "target angle");

    Serial.println(F("Motor ready."));
    Serial.println(F("Set the target angle using serial terminal:"));
    _delay(1000);
}

void loop() {
    motor.move();
    motor.loopFOC();
    command.run();
}