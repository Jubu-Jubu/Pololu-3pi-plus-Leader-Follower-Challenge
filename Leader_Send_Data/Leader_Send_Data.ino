#include "motors.h"
#include "linesensor.h"
#include "encoders.h"
#include "kinematics.h"
#include "pid.h"
#include "bumpsensor.h"

#define BUZZER_PIN 6
#define LEFT_IR 0
#define CENTRE_IR 1
#define RIGHT_IR 2
#define LEFTMOST_IR 3
#define RIGHTMOST_IR 4

#define LED_PIN 13  // Pin to activate the orange LED
boolean led_state;  // Variable to "remember" the state
// of the LED, and toggle it.



BumpSensor_c bumpsensors;
LineSensor_c lineSensors;
Kinematics_c kinematics;

PID_c speed_pid_left;
PID_c speed_pid_right;
PID_c heading;


Motors_c motors;

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
float ave_error;
float d_dist;

float theta_start;
float theta_end;
float d_theta;


bool hasRun_stop;
bool nothing;
bool hasRun_stop_dist;
bool rot_done;
bool dist_done;
bool hasRun;



void setup() {
  // put your setup code here, to run once:
  bumpsensors.init();
  

  Serial.begin(9600);
  delay(1000);
  Serial.println("***RESET***");
  

}

void loop() {
  // put your main code here, to run repeatedly:
  bumpsensors.emitter();

}
