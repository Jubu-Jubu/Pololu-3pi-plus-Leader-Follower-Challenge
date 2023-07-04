// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _BUMPSENSOR_H
#define _BUMPSENSOR_H


# define BP_LEFT_PIN  4 
# define BP_RIGHT_PIN 5


# define BUMP_SENSOR_UPDATE 50

#define EMIT 11

# define NB_BP_PINS 2

#define THRESHOLD 1900



// Class to operate the bumpsensor(s).
class BumpSensor_c {
  public:
    unsigned long elapsed_t;
    unsigned long current_ts;
    unsigned long bp_ts;

    // An array of the LEDs
    int bp_pin[NB_BP_PINS] = { BP_LEFT_PIN, BP_RIGHT_PIN};

    // This is used to select through the the LEDs
    int which;

    float sensor_read[ NB_BP_PINS ];
    // Sensor readings in an array

    bool sensor_active[ NB_BP_PINS ];

    unsigned long   elapsed_time;

    unsigned long timeout = 200000;

    // Constructor, must exist.
    BumpSensor_c() {

    }

    void init() {

      pinMode(EMIT, OUTPUT);

      for ( which = 0; which < NB_BP_PINS; which++) { //Loop Through all the light sensors

        pinMode( bp_pin[which], INPUT );
      }

      digitalWrite(EMIT, LOW);

    }
    void emitter(){
      pinMode(EMIT, OUTPUT);

      for ( which = 0; which < NB_BP_PINS; which++) { //Loop Through all the light sensors

        pinMode( bp_pin[which], INPUT );
      }

      digitalWrite(EMIT, LOW);

    }

    void emitteroff(){
      pinMode(EMIT, INPUT);

    }


    void updateSensors() {

      current_ts = millis();

      elapsed_t = current_ts - bp_ts;

      if (elapsed_t >= BUMP_SENSOR_UPDATE) {
        chargeCapacitors();

        enableIRLEDs();

        readIRLEDs();


        bp_ts = millis();
      }
    }



    void chargeCapacitors() {
      // Charge capacitor by setting input pin
      // temporarily to output and HIGH

      for ( which = 0; which < NB_BP_PINS; which++) { //Loop Through all the light sensors
        pinMode( bp_pin[which], OUTPUT );
        digitalWrite( bp_pin[which], HIGH );


      }

      // Tiny delay for capacitor to charge.
      delayMicroseconds(100);
    }


    void enableIRLEDs() {
      //  Turn input pin back to an input
      for ( which = 0; which < NB_BP_PINS; which++) { //Loop Through all the light sensors
        pinMode(bp_pin[which], INPUT);


      }
    }

    void readIRLEDs() {


      //Initialise sensor values

      for ( which = 0; which < NB_BP_PINS; which++) {
        sensor_read[which] = 0;
      }


      // Places to store microsecond count
      unsigned long start_time; // t_1
      //  unsigned long end_time;   // t_2

      int remaining = NB_BP_PINS;


      // Store current microsecond count
      start_time = micros();


      while (remaining > 0) {

        // Add code to calculate the current elapsed time
        // here.
        elapsed_time = micros() - start_time;

        for ( which = 0; which < NB_BP_PINS; which++) {

          if (digitalRead(bp_pin[which]) == LOW ) {


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

          for ( which = 0; which < NB_BP_PINS; which++) {
            sensor_read[which] = 500;
          }

          // Set remaining to 0 to force end of while()
          remaining = 0;
        }


      } // While not all light sensors are done

      for ( which = 0; which < NB_BP_PINS; which++) {


        if (sensor_read[which] > THRESHOLD) {
          sensor_active[which] = 1;
        } else {
          sensor_active[which] = 0;
        }

                Serial.print( sensor_read[which] );
        //              Serial.print("]," );
                Serial.print(",");
      }

            Serial.print("\n");

    }

};



#endif
