#include <Arduino.h>
#include <stdio.h>
#include <math.h>
#include "DCMotorController.h"
#include "commands.h"

// #define LED_PIN 8 Need to rewire to avoid pin 8

#define SILENCE_TIMEOUT 2000

// SKETCH STILL NEEDS CLEANUP

//motor control variables
float tgt_lm_speed = 0, tgt_rm_speed = 0;
float curr_lm_speed = 0, curr_rm_speed = 0; 
float global_acceleration = 1.5*pulsesPerMeter; //m/s²

unsigned long last_print_mil = 0;

void accel_subscription_callback(float global_accel)
{  
  // const std_msgs__msg__Float32 * accel_msg = (const std_msgs__msg__Float32 *)msgin;
  //Serial.printf("Received acceleration: %d\n", accel_msg->data);
  global_acceleration = global_accel*pulsesPerMeter;
}

void setup(){
  Serial.begin(115200);
  setupMotors();

  rightMotorTargetPosition = (float)rightMotor_encoder.getCount();
  leftMotorTargetPosition = (float)leftMotor_encoder.getCount();
}

// responsible for integrating the set velocity into the target position for each motor
void speedLoop(){
  float dt = ((float) (cur_micro - last_micros))/1e6; //in seconds
  float max_accel = global_acceleration*dt;
  float lm_speed_diff = curr_lm_speed - tgt_lm_speed;
  if(abs(lm_speed_diff) <= max_accel) curr_lm_speed = tgt_lm_speed;
  else if (lm_speed_diff > max_accel) curr_lm_speed-=max_accel;
  else curr_lm_speed+=max_accel;
  
  float rm_speed_diff = curr_rm_speed - tgt_rm_speed;
  if(abs(rm_speed_diff) <= max_accel) curr_rm_speed = tgt_rm_speed;
  else if (rm_speed_diff > max_accel) curr_rm_speed-=max_accel;
  else curr_rm_speed+=max_accel;

  if(abs(rightMotorTargetPosition - rightMotorPosition) < 200) rightMotorTargetPosition = rightMotorTargetPosition + curr_rm_speed*dt;
  if(abs(leftMotorTargetPosition - leftMotorPosition) < 200) leftMotorTargetPosition = leftMotorTargetPosition + curr_lm_speed*dt;
  last_micros = cur_micro;
}

unsigned int lastMotorCommand = SILENCE_TIMEOUT;

void runCommand(){
  int i = 0;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);
  switch(cmd) {
    case PING:
      Serial.printf("%ld %ld\n", arg1, arg2);
      break;
    case READ_ENCODERS:
      Serial.printf("%d %d\n", (int)leftMotorPosition, (int)rightMotorPosition);
      break;
    case RESET_ENCODERS:
      rightMotor_encoder.setCount(0);
      leftMotor_encoder.setCount(0);
      resetPID();
      Serial.println("OK");
      break;
    case MOTOR_SPEEDS:
      /* Reset the auto stop timer */
      lastMotorCommand = millis();
      tgt_lm_speed = (float)arg1;
      tgt_rm_speed = (float)arg2;
      Serial.println("OK"); 
      break;
    case MOTOR_ACCEL:
      global_acceleration = (float)arg1;
    default:
    Serial.println("Not implemented");
    break;
  }
}

void parse_command(){
  while(Serial.available()){
    chr = Serial.read();
    if (chr == 13) {
      if (arg == 1) argv1[cmd_index] = '\0';
      else if (arg == 2) argv2[cmd_index] = '\0';
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[cmd_index] = '\0';
        arg = 2;
        cmd_index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else{
        // avoid a segfault when parsing
        if(cmd_index > MAX_CMND_LENGTH){
          argv1[MAX_CMND_LENGTH] = '\0';
          argv2[MAX_CMND_LENGTH] = '\0';
        }else if (arg == 1) {
          // Subsequent arguments can be more than one character
          argv1[cmd_index] = chr;
          cmd_index++;
        }
        else if (arg == 2) {
          argv2[cmd_index] = chr;
          cmd_index++;
        }
      } 
    }
  }
}

void loop(){
  cur_micro = micros();
  // if(cur_micro - last_print_mil > 1e6){
  //   Serial.printf("LeftMot: %.01f RightMot: %.01f TargetSpd: %.01f\n", leftMotorPosition, rightMotorPosition, leftMotorOutput);
  // last_print_mil= cur_micro;
  // }
  if ((millis() - lastMotorCommand) > SILENCE_TIMEOUT) {;
    tgt_lm_speed = 0;
    tgt_rm_speed = 0;
  }
  parse_command();
  if(last_micros-cur_micro > pidSampleTime) speedLoop();
  motorsLoop();
}
