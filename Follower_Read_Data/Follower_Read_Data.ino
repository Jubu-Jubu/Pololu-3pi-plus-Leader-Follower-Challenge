#include "motors.h"
#include "linesensor.h"
#include "encoders.h"
#include "kinematics.h"
#include "pid.h"
#include "bumpsensor.h"
#include <USBCore.h>  // To fix serial print behaviour bug.
u8 USB_SendSpace(u8 ep);
#define SERIAL_ACTIVE (USB_SendSpace(CDC_TX) >= 50)

#define BUZZER_PIN 6
#define LEFT_IR 0
#define CENTRE_IR 1
#define RIGHT_IR 2
#define LEFTMOST_IR 3
#define RIGHTMOST_IR 4

#define STATE_RUNNING_EXPERIMENT 0
#define STATE_FINISHED_EXPERIMENT 1
#define RESULTS_DIM 50
int leftBumpval[RESULTS_DIM];
int rightBumpval[RESULTS_DIM];

#define LED_PIN 13  // Pin to activate the orange LED
boolean led_state;  // Variable to "remember" the state
// of the LED, and toggle it.

int state;



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

unsigned long update_ts;

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
int i = 0;



void setup() {
  // put your setup code here, to run once:
  bumpsensors.init();


  Serial.begin(9600);
  delay(1000);
  if (SERIAL_ACTIVE) Serial.println("***RESET***");
  state = STATE_RUNNING_EXPERIMENT;
  update_ts = millis();
}

void loop() {
  // put your main code here, to run repeatedly:

  if (state == STATE_RUNNING_EXPERIMENT) {

    bumpsensors.updateSensors();
    leftBumpval[i] = bumpsensors.sensor_read[0];
    rightBumpval[i] = bumpsensors.sensor_read[1];

    if (i == 50) {
      state = STATE_FINISHED_EXPERIMENT;
    }
    delay(50);
    i++;

  } else if (state == STATE_FINISHED_EXPERIMENT) {
    reportResults();
    delay(2000);
  } else {
    Serial.println("ERROR!!!!1");
  }
}


void reportResults() {
  if (SERIAL_ACTIVE) Serial.print("Time(ms): ");
  if (SERIAL_ACTIVE) Serial.println(millis());
  delay(1);
  if (SERIAL_ACTIVE) Serial.print("...\n");

  int j;

  for (j = 0; j < 50; j++) {
    if (SERIAL_ACTIVE) Serial.print(leftBumpval[j]);
    delay(1);
    if (SERIAL_ACTIVE) Serial.print(",");
    delay(1);
  }

  if (SERIAL_ACTIVE) Serial.print("...\n");
  if (SERIAL_ACTIVE) Serial.println("---End of Left Results ---\n\n");
delay(500);

  int k;

  for (k = 0; k < 50; k++) {
    if (SERIAL_ACTIVE) Serial.print(rightBumpval[k]);
    delay(1);
    if (SERIAL_ACTIVE) Serial.print(",");
    delay(1);
  }

  if (SERIAL_ACTIVE) Serial.print("...\n");
  if (SERIAL_ACTIVE) Serial.println("---End of Right Results ---\n\n");
}