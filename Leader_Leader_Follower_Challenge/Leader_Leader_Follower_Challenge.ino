#include "motors.h"
#include "linesensor.h"
#include "encoders.h"
#include "kinematics.h"
#include "pid.h"
#include "bumpsensor.h"


#define STATE_INIT 0
#define STATE_ACCEL 1
#define STATE_DECCEL 2
#define STATE_CONSTANT 3
#define STATE_ROT30 4
#define STATE_ROT45 5
#define STATE_CORNER 6
#define STATE_SPIRAL 7
#define STATE_UTURN 8
#define STATE_PARK 9









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
unsigned long for_test_ts;
unsigned long move_ts;
unsigned long turn_test_ts;

float demand;          //the set point for our motor speed
float ave_left_speed;  // Low pass filter of speed
float ave_right_speed;
int n;
int m;
int state;
bool rot_done;
bool hasRun_stop;
int f;  // Keep track of rotation


// put your setup code here, to run once:
void setup() {

  bumpsensors.init();
  lineSensors.init();
  motors.init();
  kinematics.init();
  
  bumpsensors.emitter();

  speed_pid_left.init(150.0, 0.9, -95.0);    //150.0, 0.8, -55.0
  speed_pid_right.init(150.0, 0.78, -85.0);  //150.0, 0.68, -45.0

  demand = -0.25;
  n = 0;
  m = 0;
  f = 1;

  motors.setMotorPower(0, 0);

  ave_left_speed = 0.0;
  ave_right_speed = 0.0;


  hasRun_stop = false;

  Serial.begin(9600);
  delay(3000);
  Serial.println("***RESET***");

  loop_time = millis();
  for_test_ts = millis();
  move_ts = millis();
  turn_test_ts = millis();

  speed_pid_left.reset();
  speed_pid_right.reset();
}


// put your main code here, to run repeatedly:
void loop() {
  kinematics.update();
  updateState();

  Serial.print("State : '");
  Serial.print(state);
  Serial.println("'");

  long for_ts;
  long turn_ts;
  for_ts = millis() - for_test_ts;


  if (for_ts >= 1500) {
    n = 1;
    if (for_ts <= 2000) {
      n = 2;
      for_test_ts = millis();
    }
  }
}

void updateState() {
  if (state == STATE_INIT) {
    moveIt();
  } else if (state == STATE_ACCEL) {
    accel(0.6);
  } else if (state == STATE_DECCEL) {
    deccel(0.25);
  } else if (state == STATE_ROT30) {
    move30();
  } else if (state == STATE_ROT45) {
    move45();
  }else if (state == STATE_CORNER) {
  move90();
  }else if (state == STATE_CONSTANT) {
  moveIt2();
  }else if (state == STATE_UTURN) {
  uturn();
  }else if (state == STATE_PARK) {
  park();
  }
}


void uturn(){
  long del_ts;
    goFoward(-0.2, -0.35);
    del_ts = millis() - move_ts;
    if (del_ts >= 5600) {
      state = STATE_PARK;
      move_ts = millis();
    }
}

void move90(){
  long del_ts;
    goFoward(-0.2, -0.35);
    del_ts = millis() - move_ts;
    if (del_ts >= 2800) {
      state = STATE_CONSTANT;
      move_ts = millis();
    }
}

void move45() {

  if (f == 1) {
    rotate(-45);
    if (rot_done) {
      motors.stopRobot();
      f = 2;
    }
  }

  if (f == 2) {
    // moveIt();
    long del_ts;
    goFoward(-0.25, -0.25);
    del_ts = millis() - move_ts;
    if (del_ts >= 1500) {
      f = 3;  //maintain for everything
      move_ts = millis();
    }
  }

  if (f == 3) {
    rotate(45);
    if (rot_done) {
      motors.stopRobot();
      f = 4;
    }
  }

  if (f == 4) {
    long del_ts;
    goFoward(-0.25, -0.25);
    del_ts = millis() - move_ts;
    if (del_ts >= 2500) {
      f = 1 ;  //maintain for everything
      state = STATE_CORNER;
      move_ts = millis();
    }
  }
  
}



void move30() {



  if (f == 1) {
    rotate(30);
    if (rot_done) {
      motors.stopRobot();
      f = 2;
    }
  }

  if (f == 2) {
    // moveIt();
    long del_ts;
    goFoward(-0.25, -0.25);
    del_ts = millis() - move_ts;
    if (del_ts >= 1500) {
      f = 3;  //maintain for everything
      move_ts = millis();
    }
  }



  if (f == 3) {
    rotate(-30);
    if (rot_done) {
      motors.stopRobot();
      f = 4;
    }
  }

  if (f == 4) {
    long del_ts;
    goFoward(-0.25, -0.25);
    del_ts = millis() - move_ts;
    if (del_ts >= 2000) {
      f = 1 ;  //maintain for everything
      state = STATE_ROT45;
      move_ts = millis();
    }
  }
  
}

void rotate(float angle) {
  float theta_start;
  float theta_end;
  float d_theta;
  rot_done = false;
  // kinematics.init();


  angle = angle * (kinematics.pi / 180);

  if (!hasRun_stop) {
    motors.stopRobot();
    theta_start = kinematics.thetai;
    hasRun_stop = true;
  } else {

    theta_end = kinematics.thetai;
  }

  d_theta = abs(theta_end - theta_start);

  if (angle > 0) {
    if (d_theta < abs(angle)) {
      motors.setMotorPower(25, -25);
    } else {
      motors.stopRobot();
      rot_done = true;
    }
  } else {
    if (d_theta < abs(angle)) {
      motors.setMotorPower(-25, 25);
    } else {
      motors.stopRobot();
      rot_done = true;
    }
  }

  if (rot_done) {
    hasRun_stop = false;
  }
}

void deccel(float vel) {
  long del_ts;
  goFoward(-vel, -vel);
  del_ts = millis() - move_ts;
  if (del_ts >= 2500) {
    state = STATE_ROT30;
    move_ts = millis();
  }
}
void accel(float vel) {
  long del_ts;
  goFoward(-vel, -vel);
  del_ts = millis() - move_ts;
  if (del_ts >= 2500) {
    state = STATE_DECCEL;
    move_ts = millis();
  }
}
void moveIt() {
  long del_ts;
  goFoward(-0.25, -0.25);
  del_ts = millis() - move_ts;
  if (del_ts >= 1500) {
    state = STATE_ACCEL;
    move_ts = millis();
  }
}
void park(){
  long del_ts;
 
  del_ts = millis() - move_ts;
  if (del_ts >= 2500) {
    motors.stopRobot();
    bumpsensors.emitteroff();
  }else{
   goFoward(-0.25, -0.25);
  }
}

void moveIt2() {
  long del_ts;
  goFoward(-0.25, -0.25);
  del_ts = millis() - move_ts;
  if (del_ts >= 2500) {
    state = STATE_UTURN;
    move_ts = millis();
  }
}

void goFoward(float demandL, float demandR) {

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