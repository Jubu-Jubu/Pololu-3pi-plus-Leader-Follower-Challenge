// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _LINESENSOR_H
#define _LINESENSOR_H


# define LS_LEFT_PIN   A0
# define LS_CENTRE_PIN A2
# define LS_RIGHT_PIN A3
# define LS_LEFTMOST_PIN A11
# define LS_RIGHTMOST_PIN A4



# define LINE_SENSOR_UPDATE 50

#define EMIT 11

# define NB_LS_PINS 5

#define THRESHOLD 1900



// Class to operate the linesensor(s).
class LineSensor_c {
  public:
    unsigned long elapsed_t;
    unsigned long current_ts;
    unsigned long ls_ts;

    // An array of the LEDs
    int ls_pin[NB_LS_PINS] = { LS_LEFT_PIN, LS_CENTRE_PIN, LS_RIGHT_PIN, LS_LEFTMOST_PIN, LS_RIGHTMOST_PIN };

    // This is used to select through the the LEDs
    int which;

    float sensor_read[ NB_LS_PINS ];
    // Sensor readings in an array

    bool sensor_active[ NB_LS_PINS ];

    unsigned long   elapsed_time;

    unsigned long timeout = 5000;

    // Constructor, must exist.
    LineSensor_c() {

    }

    void init() {

      pinMode(EMIT, OUTPUT);

      for ( which = 0; which < NB_LS_PINS; which++) { //Loop Through all the light sensors

        pinMode( ls_pin[which], INPUT );
      }

      digitalWrite(EMIT, HIGH);

    }


    void updateSensors() {

      current_ts = millis();

      elapsed_t = current_ts - ls_ts;

      if (elapsed_t >= LINE_SENSOR_UPDATE) {
        chargeCapacitors();

        enableIRLEDs();

        readIRLEDs();


        ls_ts = millis();
      }
    }



    void chargeCapacitors() {
      // Charge capacitor by setting input pin
      // temporarily to output and HIGH

      for ( which = 0; which < NB_LS_PINS; which++) { //Loop Through all the light sensors
        pinMode( ls_pin[which], OUTPUT );
        digitalWrite( ls_pin[which], HIGH );


      }

      // Tiny delay for capacitor to charge.
      delayMicroseconds(10);
    }


    void enableIRLEDs() {
      //  Turn input pin back to an input
      for ( which = 0; which < NB_LS_PINS; which++) { //Loop Through all the light sensors
        pinMode(ls_pin[which], INPUT);


      }
    }

    void readIRLEDs() {


      //Initialise sensor values

      for ( which = 0; which < NB_LS_PINS; which++) {
        sensor_read[which] = 0;
      }


      // Places to store microsecond count
      unsigned long start_time; // t_1
      //  unsigned long end_time;   // t_2

      int remaining = NB_LS_PINS;


      // Store current microsecond count
      start_time = micros();


      while (remaining > 0) {

        // Add code to calculate the current elapsed time
        // here.
        elapsed_time = micros() - start_time;

        for ( which = 0; which < NB_LS_PINS; which++) {

          if (digitalRead(ls_pin[which]) == LOW ) {


            if (sensor_read[which] == 0) {
              sensor_read[which] = (float) elapsed_time;

              // Print output.
              //              Serial.print("[" );
              //              Serial.print(which );
              //              Serial.print("," );
              //              Serial.print( sensor_read[which] );
              //              Serial.print("," );


              remaining = remaining - 1;
            }
          }


        }

        if ( elapsed_time >= timeout ) {

          // Here, you may need to set an appropriate
          // sensor_read[ which ] value to indicate a
          // timeout.  An appropriate initial value of
          // sensor_read[] could mean this step isn't
          // necessary.

          for ( which = 0; which < NB_LS_PINS; which++) {
            sensor_read[which] = 0;
          }

          // Set remaining to 0 to force end of while()
          remaining = 0;
        }


      } // While not all light sensors are done

      for ( which = 0; which < NB_LS_PINS; which++) {


        if (sensor_read[which] > THRESHOLD) {
          sensor_active[which] = 1;
        } else {
          sensor_active[which] = 0;
        }

//        Serial.print( sensor_read[which] );
        //              Serial.print("]," );
//        Serial.print(",");
      }

//      Serial.print("\n");

    }

};



#endif
