#include "motors.h"
#include "linesensor.h"
#include "encoders.h"
#include "kinematics.h"
#include "pid.h"
#include "bumpsensor.h"
#include <USBCore.h>  // To fix serial print behaviour bug.

u8 USB_SendSpace(u8 ep);

#define SERIAL_ACTIVE (USB_SendSpace(CDC_TX) >= 50)
#define STATE_RUNNING_EXPERIMENT 0
#define STATE_FINISHED_EXPERIMENT 1
#define LED_PIN 13  // Pin to activate the orange LED
boolean led_state;  // Variable to "remember" the state
// of the LED, and toggle it.





BumpSensor_c bumpsensors;
LineSensor_c lineSensors;
Kinematics_c kinematics;

PID_c speed_pid_left;
PID_c speed_pid_right;
PID_c follow;
PID_c turning;

Motors_c motors;

int bumpError[50];
int state;

long last_left;    //Change in count
long last_right;   //Change in count
unsigned long dt;  //Difference in time of loop
volatile double left_vel;
volatile double right_vel;
unsigned long loop_time;
unsigned long pid_test_ts;

float demand;          //the set point for our motor speed
float ave_left_speed;  // Low pass filter of speed
float ave_right_speed;
float last_difference;  // For low pass filter

int i = 0;


// put your setup code here, to run once:
void setup() {

  bumpsensors.init();
  motors.init();
  kinematics.init();

  // speed_pid_left.init(150.0, 0.9, -95.0);    //150.0, 0.8, -55.0
  // speed_pid_right.init(150.0, 0.78, -85.0);  //150.0, 0.68, -45.0

  speed_pid_left.init(120.0, 0.28, -20.0);
  speed_pid_right.init(120.0, 0.3, -20.0);

  follow.init(9.0, 0.0, -5.0);
  turning.init(0.0004, 0.0, 0.0);

  demand = 0.25;

  motors.setMotorPower(0, 0);

  ave_left_speed = 0.0;
  ave_right_speed = 0.0;



  Serial.begin(9600);
  delay(500);
  Serial.println("***RESET***");

  if (SERIAL_ACTIVE) Serial.println("***RESET***");

  state = STATE_RUNNING_EXPERIMENT;

  loop_time = millis();
  pid_test_ts = millis();

  speed_pid_left.reset();
  speed_pid_right.reset();
  follow.reset();
  turning.reset();
}


// put your main code here, to run repeatedly:
void loop() {

  if (state == STATE_RUNNING_EXPERIMENT) {

    bumpsensors.updateSensors();
    int leftBump;
    int rightBump;
    float average;
    float for_drive;
    int setPoint = 170;
    int change;
    int error;
  

    leftBump = bumpsensors.sensor_read[0];
    rightBump = bumpsensors.sensor_read[1];
    average = (leftBump + rightBump) / 2;
    error = leftBump - rightBump;

    change = millis() - pid_test_ts;
  

    if (average != 0) {
      float difference;
      float left_drive;
      float right_drive;
      float turn_drive;

      difference = setPoint - average;
      difference = (last_difference * 0.7) + (difference * 0.3);



      for_drive = follow.update(0.0, difference);
      turn_drive = turning.update(0.0, error);




      last_difference = difference;
  

      for_drive = for_drive * 0.00007;
      left_drive = left_drive * 0.0001;

      right_drive = left_drive * 0.0001;

      left_drive = for_drive - turn_drive;
      right_drive = for_drive + turn_drive;



      // Serial.print(left_drive);
      // Serial.print(",");
      // Serial.println(right_drive);

      // float forward_vel;
      // forward_vel = 2;
      // forward_vel = for_drive * forward_vel;
      goForward(left_drive, right_drive);
    } else {
      motors.stopRobot();
      speed_pid_left.reset();
      speed_pid_right.reset();
      follow.reset();
      turning.reset();
    }
  } else if (state == STATE_FINISHED_EXPERIMENT) {

    reportResults();
    delay(2000);
  } else {
    Serial.println("ERROR!!!!");
  }
}


void reportResults() {

  if (SERIAL_ACTIVE) Serial.print("Time(ms): ");
  if (SERIAL_ACTIVE) Serial.println(millis());
  delay(1);
  if (SERIAL_ACTIVE) Serial.print("...\n");

  int j;

  for (j = 0; j < 50; j++) {
    if (SERIAL_ACTIVE) Serial.print(bumpError[j]);
    delay(1);
    if (SERIAL_ACTIVE) Serial.print(",");
    delay(1);
  }

  if (SERIAL_ACTIVE) Serial.print("...\n");
  if (SERIAL_ACTIVE) Serial.println("---End of Error Results ---\n\n");
  delay(500);
}


void goForward(float demandL, float demandR) {

  dt = millis() - loop_time;
  if (dt > 20) {

    long d_count_left;
    long d_count_right;
    float left_speed;
    float right_speed;

    loop_time = millis();

    d_count_left = count_eLeft - last_left;
    d_count_right = count_eRight - last_right;
    last_left = count_eLeft;
    last_right = count_eRight;

    left_speed = (float)d_count_left;
    left_speed /= (float)dt;
    right_speed = (float)d_count_right;
    right_speed /= (float)dt;

    ave_left_speed = (ave_left_speed * 0.7) + (left_speed * 0.3);
    ave_right_speed = (ave_right_speed * 0.7) + (right_speed * 0.3);

    //    Serial.print(left_speed * 100);
    //    Serial.print(",");
    //    Serial.println(ave_left_speed * 100);

    float pwmL;
    float pwmR;


    pwmL = speed_pid_left.update(demandL, ave_left_speed);
    pwmR = speed_pid_right.update(demandR, ave_right_speed);

    motors.setMotorPower(pwmL, pwmR);
  }
}