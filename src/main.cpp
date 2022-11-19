#include <micro_ros_arduino.h>

#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <geometry_msgs/msg/twist.h>

#include <math.h>
#include "DCMotorController.h"

#define WIFI_SSID ""
#define WIFI_PWD  ""

#define LED_PIN 2 

rcl_subscription_t cmd_vel_subscriber;
rcl_subscription_t accel_subscriber;
rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
geometry_msgs__msg__Twist sub_msg;
std_msgs__msg__Float32 accel_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

float targetLinearVel = 0, targetAngularVel = 0, tgt_lm_speed = 0, tgt_rm_speed = 0;
float curr_lm_speed = 0, curr_rm_speed = 0; 
float global_acceleration = 1.5*pulsesPerMeter; //m/s²

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  for (size_t i = 0; i < 100; i++)
  {
    //digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
  ESP.restart();
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    msg.data++;
  }
}

void twist_subscription_callback(const void * msgin)
{  
  const geometry_msgs__msg__Twist * sub_msg = (const geometry_msgs__msg__Twist *)msgin;
  targetAngularVel = sub_msg->angular.z;
  targetAngularVel = targetAngularVel*105.0f;
  targetLinearVel = (float)sub_msg->linear.x; //meters per second
  targetLinearVel = (float)(targetLinearVel)*pulsesPerMeter; //pulses per second
  tgt_rm_speed = constrain((-targetLinearVel + targetAngularVel), -maxPulsesPerSecond,maxPulsesPerSecond);
  tgt_lm_speed = constrain((+targetLinearVel + targetAngularVel), -maxPulsesPerSecond,maxPulsesPerSecond);
}

void accel_subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Float32 * accel_msg = (const std_msgs__msg__Float32 *)msgin;
  //Serial.printf("Received acceleration: %d\n", accel_msg->data);
  global_acceleration = accel_msg->data*pulsesPerMeter;
}

void setup_ros_sub(){
  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &cmd_vel_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  RCCHECK(rclc_subscription_init_default(
    &accel_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "stepper_accel"));
  
  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &sub_msg, &twist_subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &accel_subscriber, &accel_msg, &accel_subscription_callback, ON_NEW_DATA));
}

void setup_ros_pub(){
  //create publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "step_pos"));
    msg.data = 0;
}



void setup(){
  // Serial.begin(115200);
  set_microros_transports();
  setupMotors();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  delay(1000);

  allocator = rcl_get_default_allocator();
  // Initialize and modify options (Set DOMAIN ID to 30)
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, 30));

  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
  
  // create node
  RCCHECK(rclc_node_init_default(&node, "stepper32", "", &support));
  setup_ros_pub();
  setup_ros_sub();
  rightMotorTargetPosition = (float)rightMotor_encoder.getCount();
  leftMotorTargetPosition = (float)leftMotor_encoder.getCount();
}


/*
 Linear to angular: 
  RightMotorSpeed = -LeftMotorSpeed
  rad/s to mm/s = rad/s*105mm
*/

void speedLoop(){
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
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

void loop(){
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0)));
  cur_micro = micros();
  if(cur_micro - last_print_mil > 1e6){
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  //   Serial.printf("LeftMot: %.01f RightMot: %.01f TargetSpd: %.01f\n", leftMotorPosition, rightMotorPosition, leftMotorOutput);
    last_print_mil= cur_micro;
  }
  if(last_micros-cur_micro > 1e3) speedLoop();
  motorsLoop();
  
}
