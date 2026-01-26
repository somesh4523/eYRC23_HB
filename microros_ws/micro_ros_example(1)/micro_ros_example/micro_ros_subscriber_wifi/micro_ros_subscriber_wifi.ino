#include <micro_ros_arduino.h>
#include <ESP32Servo.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/wrench.h>

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

Servo servo1;
Servo servo2;
Servo servo3;
Servo micro_servo;

#define LED_PIN 13
#define servo1_pin  12
#define servo2_pin  14
#define servo3_pin  27
#define micro_servo_pin   26

void servo_init(){
  servo1.attach(servo1_pin);
  servo2.attach(servo2_pin);
  servo3.attach(servo3_pin);
  micro_servo.attach(micro_servo_pin);
}

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void subscription_callback(const void * msgin)
{  
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  // digitalWrite(LED_PIN, (msg->data == 0) ? LOW : HIGH);  
}

void setup() {
  set_microros_wifi_transports("soma", "54232221", "192.168.43.170", 8888);
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);
   servo_init();

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "hb_controller", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel/bot_1"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  Serial.print("subscribing data :");
  // Serial.println(msg.linear.x);
  float Vx = msg.linear.x;
  float Vy =  msg.linear.y;
  float Wz = msg.angular.z;

  // Replace this with the actual distance from the center of the robot to the wheels
  float R = 1.0;

  // Calculate wheel velocities
  float V1 = Vx - Vy - Wz * R;
  float V2 = 0.5 * Vx + sqrt(3)/2 * Vy + Wz * R;
  float V3 = 0.5 * Vx - sqrt(3)/2 * Vy + Wz * R;

   // Convert wheel velocities to servo angles
  int angle1 = map(V1, -1, 1, 0, 180);
  int angle2 = map(V2, -1, 1, 0, 180);
  int angle3 = map(V3, -1, 1, 0, 180);

  // Command servos
  servo1.write(angle1);
  servo2.write(angle2);
  servo3.write(angle3);

  Serial.println(angle1);
  Serial.println(angle2);
  Serial.println(angle3);
  Serial.println("HEREE");


}
