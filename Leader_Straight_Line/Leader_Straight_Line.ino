#include "motors.h"
#include "linesensor.h"
#include "encoders.h"
#include "kinematics.h"
#include "pid.h"
#include "bumpsensor.h"

#define LED_PIN 13  // Pin to activate the orange LED
boolean led_state;  // Variable to "remember" the state
// of the LED, and toggle it.





BumpSensor_c bumpsensors;
LineSensor_c lineSensors;
Kinematics_c kinematics;

PID_c speed_pid_left;
PID_c speed_pid_right;

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


// put your setup code here, to run once:
void setup() {

  bumpsensors.init();
  lineSensors.init();
  motors.init();
  kinematics.init();

  speed_pid_left.init(150.0, 0.9, -95.0);    //150.0, 0.8, -55.0
  speed_pid_right.init(150.0, 0.78, -85.0);  //150.0, 0.68, -45.0

  demand = -0.25;

  motors.setMotorPower(0, 0);

  ave_left_speed = 0.0;
  ave_right_speed = 0.0;


  loop_time = millis();
  pid_test_ts = millis();

  Serial.begin(9600);
  delay(3000);
  Serial.println("***RESET***");

  // Set LED pin as an output
  pinMode(LED_PIN, OUTPUT);

  // Set initial state of the LED
  led_state = false;

  speed_pid_left.reset();
  speed_pid_right.reset();
}


// put your main code here, to run repeatedly:
void loop() {
  bumpsensors.emitter();

  goFoward(demand);
  long test_ts;
  test_ts = millis() - pid_test_ts;

  if (test_ts > 1000000) {
    pid_test_ts = millis();
    // demand = demand * -1;
    motors.stopRobot();
  }
}


void goFoward(float demand) {

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


    pwmL = speed_pid_left.update(demand, ave_left_speed);
    pwmR = speed_pid_right.update(demand, ave_right_speed);

    motors.setMotorPower(pwmL, pwmR);
  }
}