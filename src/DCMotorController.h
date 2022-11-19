#include "driver/ledc.h"

#include <ESP32Encoder.h>

#include "PID_v1.h"


#define MOTOR_PWM_FREQUENCY 18000
#define MOTOR_PWM_RESOLUTION_BITS 10

// PIN DEFINITIONS
const int potPin = 4;

const int rightMotor_SenseGreen = 35;
const int rightMotor_SenseYellow = 34;
const int rightMotor_Dir = 33;
const int rightMotor_Enable = 32;
ESP32Encoder rightMotor_encoder;

const int leftMotor_SenseGreen = 13;
const int leftMotor_SenseYellow = 15;
const int leftMotor_Dir = 27;
const int leftMotor_Enable = 14;
ESP32Encoder leftMotor_encoder;

// Motor constants 
const int rightMotor_Channel = 0;
const int leftMotor_Channel = 2;

//PhysicalConstants (these may be wise to put under a service)
const float pulsesPerRev = 898.0;
const float wheelDiameter = 0.068; //in meters
const float maxPulsesPerSecond = 3000;
const float metersPerRev = M_PI*wheelDiameter;
const float pulsesPerMeter = pulsesPerRev/metersPerRev;
const float metersPerPulse = metersPerRev/pulsesPerRev;
const float maxSpeed = maxPulsesPerSecond*metersPerPulse; //metersPerSecond

//PID
const int pidSampleTime = 1000; //microsseconds
float Kp = 40.0;
float Kd = 0.0;
float Ki = 10.0;

float rightMotorTargetPosition = 0;
float rightMotorPosition, rightMotorOutput;
PID rightMotorPID(&rightMotorPosition, &rightMotorOutput, &rightMotorTargetPosition, Kp, Ki, Kd, DIRECT);

float leftMotorTargetPosition = 0;
float leftMotorPosition, leftMotorOutput;
PID leftMotorPID(&leftMotorPosition, &leftMotorOutput, &leftMotorTargetPosition, Kp, Ki, Kd, DIRECT);

int64_t last_pos, cur_pos;
unsigned long last_micros, cur_micro, last_print_mil;




void setup_ledc_channel(uint8_t pin, uint8_t chan, uint8_t inverted){
    if(chan >= SOC_LEDC_CHANNEL_NUM<<1){
        return;
    }
    uint8_t group=(chan/8), channel=(chan%8), timer=((chan/2)%4);
    ledc_channel_config_t ledc_channel = {
        .gpio_num       = pin,
        .speed_mode     = (ledc_mode_t)group,
        .channel        = (ledc_channel_t)channel,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = (ledc_timer_t)timer,
        .duty           = 0,
        .hpoint         = 0,
        .flags          = { .output_invert = inverted }
    };
    ledc_channel_config(&ledc_channel);
}

void setMotorOutput(int contollerOutput, int motorChannel, int motorDir){
  digitalWrite(motorDir, contollerOutput > 0);
  ledcWrite(motorChannel, abs(contollerOutput));
}

void setupMotors(){
  //Right Motor Setup
  pinMode(rightMotor_Dir,OUTPUT);
  digitalWrite(rightMotor_Dir, 0);
  ledcSetup(rightMotor_Channel, MOTOR_PWM_FREQUENCY, MOTOR_PWM_RESOLUTION_BITS);
  setup_ledc_channel(rightMotor_Enable, rightMotor_Channel, 0);
  rightMotor_encoder.attachFullQuad(rightMotor_SenseGreen, rightMotor_SenseYellow);
  rightMotor_encoder.clearCount();
  rightMotorPID.SetMode(AUTOMATIC);
  rightMotorPID.SetSampleTime(pidSampleTime);
  rightMotorPID.SetOutputLimits(-2<<(MOTOR_PWM_RESOLUTION_BITS-1), 2<<(MOTOR_PWM_RESOLUTION_BITS-1));  


  // Left Motor Setup
  pinMode(leftMotor_Dir,OUTPUT);
  digitalWrite(leftMotor_Dir, 0);
  ledcSetup(leftMotor_Channel, MOTOR_PWM_FREQUENCY, MOTOR_PWM_RESOLUTION_BITS);
  setup_ledc_channel(leftMotor_Enable, leftMotor_Channel, 0);
  leftMotor_encoder.attachFullQuad(leftMotor_SenseGreen, leftMotor_SenseYellow);
  leftMotor_encoder.clearCount();
  leftMotorPID.SetMode(AUTOMATIC);
  leftMotorPID.SetSampleTime(pidSampleTime);
  leftMotorPID.SetOutputLimits(-2<<(MOTOR_PWM_RESOLUTION_BITS-1), 2<<(MOTOR_PWM_RESOLUTION_BITS-1));  
  
  last_pos = rightMotor_encoder.getCount();
  cur_pos = last_pos;
}

void motorsLoop(){
  rightMotorPosition = (float)rightMotor_encoder.getCount();
  rightMotorPID.Compute();
  setMotorOutput((int)rightMotorOutput, rightMotor_Channel, rightMotor_Dir);

  leftMotorPosition = (float)leftMotor_encoder.getCount();
  leftMotorPID.Compute();
  setMotorOutput((int)leftMotorOutput, leftMotor_Channel, leftMotor_Dir);
}
