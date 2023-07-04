// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _MOTORS_H
#define _MOTORS_H


# define L_PWM_PIN 10
# define L_DIR_PIN 16
# define R_PWM_PIN 9
# define R_DIR_PIN 15

# define FWD LOW
# define REV HIGH
# define MOTOR_UPDATE       20
#define MAX_PWM 50



// Class to operate the motor(s).
class Motors_c {
  public:

    unsigned long elapsed_t;
    unsigned long motor_ts;
    float pwmL = 0;
    float pwmR = 0;
    // Constructor, must exist.
    Motors_c() {

    }

    // Use this function to
    // initialise the pins and
    // state of your motor(s).
    void init() {

      //Setting the motor pins as output
      pinMode( L_PWM_PIN, OUTPUT );
      pinMode( L_DIR_PIN, OUTPUT );
      pinMode( R_PWM_PIN, OUTPUT );
      pinMode( R_DIR_PIN, OUTPUT );



      // Set initial direction (HIGH/LOW)
      // for the direction pins.
      // ...

      digitalWrite(L_DIR_PIN, FWD);
      digitalWrite(R_DIR_PIN, FWD);

      // Set initial power values for the PWM
      // Pins.
      // ...

      analogWrite(L_PWM_PIN, 0);
      analogWrite(R_PWM_PIN, 0);


    }

    // Write a function to operate
    // your motor(s)
    // ...


    void setMotorPower( float left_pwm, float right_pwm ) {

      if ( left_pwm < 0) {
        digitalWrite(L_DIR_PIN, REV);
      } else {
        digitalWrite(L_DIR_PIN, FWD);
      }

      if ( right_pwm < 0) {
        digitalWrite(R_DIR_PIN, REV);
      } else {
        digitalWrite(R_DIR_PIN, FWD);
      }

      if (abs(left_pwm) < 15) {
        left_pwm = 0;
      }

      if (abs(left_pwm) > MAX_PWM) {
        left_pwm = MAX_PWM;
      }

      if (abs(right_pwm) < 15) {
        right_pwm = 0;
      }

      if (abs(right_pwm) > MAX_PWM) {
        right_pwm = MAX_PWM;
      }


      elapsed_t = millis() - motor_ts;

      if (elapsed_t >= MOTOR_UPDATE) {

        analogWrite( L_PWM_PIN, abs(left_pwm));

        analogWrite( R_PWM_PIN, abs(right_pwm));
        motor_ts = millis();

      }


    }

    void stopRobot() {
      analogWrite( L_PWM_PIN, 0);
      analogWrite( R_PWM_PIN, 0);
    }

};



#endif
