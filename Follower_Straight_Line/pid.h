// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _PID_H
#define _PID_H

// Class to contain generic PID algorithm.
class PID_c {
  public:

    float last_error;
    float p_term;
    float i_term;
    float d_term;
    float i_sum;
    float feedback;

    float kp;
    float ki;
    float kd;

    // To determine the elapsed time
    unsigned long ms_last_ts;



    // Constructor, must exist.
    PID_c() {

    }


    void init(float p, float i, float d ) {


      p_term = 0;
      i_term = 0;
      d_term = 0;
      i_sum = 0;
      last_error = 0;
      feedback = 0;

      kp = p;
      ki = i;
      kd = d;

      ms_last_ts = millis();
    }
    //Reset all the variables to avoid integral wind up

    void reset() {
      p_term = 0;
      i_term = 0;
      d_term = 0;
      i_sum = 0;
      last_error = 0;
      feedback = 0;


      ms_last_ts = millis();
    }



    float update(float demand, float measurement) {
      float error;
      unsigned long ms_now_ts;
      unsigned long ms_dt;
      float float_dt;
      float diff_error;

      ms_now_ts = millis();
      ms_dt = ms_now_ts - ms_last_ts;

      // Update ms_lasttimestamp

      ms_last_ts = millis();

      float_dt = (float)ms_dt;

      if (float_dt == 0) {
        return feedback;
      }
      error = demand - measurement;



      p_term = kp * error;

      i_sum = i_sum + (error * float_dt);

      i_term = ki * i_sum;

      diff_error = (error - last_error) / float_dt;
      last_error = error;

      d_term = kd * diff_error;


      feedback = p_term + i_term + d_term;

      return feedback;
    }

};



#endif
