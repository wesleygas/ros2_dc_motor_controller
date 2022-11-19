#include <micro_ros_arduino.h>
#include <stdio.h>
#include <math.h>
#include "DCMotorController.h"

void setup(){
  Serial.begin(115200);
  setupMotors();
}

void loop(){
  cur_micro = micros();
  if(cur_micro - last_print_mil > 1e4){
    Serial.printf("LeftMot: %.01f RightMot: %.01f TargetSpd: %.01f\n", leftMotorPosition, rightMotorPosition, leftMotorOutput);
    last_print_mil= cur_micro;
  }
  if(cur_micro - last_micros > 1e3){
    // targetSpeed = (analogRead(potPin) >> 1) - 1023;
    // if(abs(targetSpeed) < 50) targetSpeed = 0;
    targetSpeed = std::sin((float)cur_micro/2e6)*500; //meters per second
    targetSpeed = (float)(targetSpeed)*pulsesPerMeter*1e-3f; //meters per second
    targetSpeed = constrain(targetSpeed,-maxPulsesPerSecond,maxPulsesPerSecond);
    float dt = ((float) (cur_micro - last_micros))/1e6; //in seconds
    if(abs(rightMotorTargetPosition - rightMotorPosition) < 200) rightMotorTargetPosition = rightMotorTargetPosition + targetSpeed*dt;
    if(abs(leftMotorTargetPosition - leftMotorPosition) < 200) leftMotorTargetPosition = leftMotorTargetPosition + targetSpeed*dt;
    last_micros = cur_micro;
  }
  motorsLoop();
  
}
