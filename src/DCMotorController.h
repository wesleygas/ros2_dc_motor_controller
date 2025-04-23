#include "driver/ledc.h"
#include "SoftwareEncoder.h"

#include "PID_v1.h"


#define MOTOR_PWM_FREQUENCY 18000
#define MOTOR_PWM_RESOLUTION_BITS 10


const int leftMotor_SenseGreen = 7;
const int leftMotor_SenseYellow = 2;
const int leftMotor_A_out = 0;
const int leftMotor_B_out = 1;
SoftwareEncoder leftMotor_encoder(leftMotor_SenseGreen, leftMotor_SenseYellow);

const int rightMotor_SenseGreen = 8;
const int rightMotor_SenseYellow = 3;
const int rightMotor_A_out = 10;
const int rightMotor_B_out = 9;
SoftwareEncoder rightMotor_encoder(rightMotor_SenseGreen, rightMotor_SenseYellow);

// Motor constants 
const int rightMotor_Channel = 0;
const int leftMotor_Channel = 2; // the channel 1 cannot be used because it shares the timer with right motor channel

//PhysicalConstants (these may be wise to put under a service)
const float pulsesPerRev = 898.0;
const float wheelDiameter = 0.068; //in meters
const float maxPulsesPerSecond = 3000;
const float metersPerRev = M_PI*wheelDiameter;
const float pulsesPerMeter = pulsesPerRev/metersPerRev;
const float metersPerPulse = metersPerRev/pulsesPerRev;
const float maxSpeed = maxPulsesPerSecond*metersPerPulse; //metersPerSecond
const float diffPulsePerRad = 105;

//PID
const int pidSampleTime = 1000; //microsseconds -> 1kHz
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
unsigned long last_micros, cur_micro;




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

void setMotorOutput(int contollerOutput, int motorChannel){
  if(contollerOutput > 0){
    ledcWrite(motorChannel, abs(contollerOutput));
    ledcWrite(motorChannel+1, 0);
  } else {
    ledcWrite(motorChannel, 0);
    ledcWrite(motorChannel+1, abs(contollerOutput));
  }
}

void setupMotors(){
  //Right Motor Setup
  ledcSetup(rightMotor_Channel, MOTOR_PWM_FREQUENCY, MOTOR_PWM_RESOLUTION_BITS);
  setup_ledc_channel(rightMotor_A_out, rightMotor_Channel, 0);
  setup_ledc_channel(rightMotor_B_out, rightMotor_Channel + 1, 0);
  rightMotor_encoder.setCount(0);
  rightMotorPID.SetMode(AUTOMATIC);
  rightMotorPID.SetSampleTime(pidSampleTime);
  rightMotorPID.SetOutputLimits(-2<<(MOTOR_PWM_RESOLUTION_BITS-1), 2<<(MOTOR_PWM_RESOLUTION_BITS-1));  


  // Left Motor Setup
  ledcSetup(leftMotor_Channel, MOTOR_PWM_FREQUENCY, MOTOR_PWM_RESOLUTION_BITS);
  setup_ledc_channel(leftMotor_A_out, leftMotor_Channel, 0);
  setup_ledc_channel(leftMotor_B_out, leftMotor_Channel + 1, 0);
  leftMotor_encoder.setCount(0);
  leftMotorPID.SetMode(AUTOMATIC);
  leftMotorPID.SetSampleTime(pidSampleTime);
  leftMotorPID.SetOutputLimits(-2<<(MOTOR_PWM_RESOLUTION_BITS-1), 2<<(MOTOR_PWM_RESOLUTION_BITS-1));  
  
  last_pos = rightMotor_encoder.getCount();
  cur_pos = last_pos;
}

void resetPID(){
  leftMotorTargetPosition = 0;
  leftMotorPosition = 0;
  leftMotorPID.ResetPID();
  rightMotorTargetPosition = 0;
  rightMotorPosition = 0;
  rightMotorPID.ResetPID();
}

void motorsLoop(){
  rightMotorPosition = (float)rightMotor_encoder.getCount();
  rightMotorPID.Compute();
  setMotorOutput((int)rightMotorOutput, rightMotor_Channel);

  leftMotorPosition = (float)leftMotor_encoder.getCount();
  leftMotorPID.Compute();
  setMotorOutput((int)leftMotorOutput, leftMotor_Channel);
}
