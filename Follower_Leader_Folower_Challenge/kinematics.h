// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _KINEMATICS_H
#define _KINEMATICS_H

# define KINEMATICS_UPDATE  10
#define CPR 343.3

// Class to track robot position.
class Kinematics_c {
  public:

    //Odometry
    double xi;
    double yi;
    double thetai;
    double xr;
    double yr;
    double thetar;
    long prev_eLeft;
    long prev_eRight;
    double radius = 16.5;
    double leg = 42.1;//(90-7)/2

    unsigned long kine_ts;

    float dpr;

    float pi = 3.1415;

    // Constructor, must exist.
    Kinematics_c() {

    }

    void init() {
      setupEncoderRight();
      setupEncoderLeft();

      xi = 0;
      yi = 0;


      dpr = (pi * 2 * radius) / CPR;
    }

    // Use this function to update
    // your kinematics
    void update() {

      unsigned long e_ts;
      float phiLeft;
      float phiRight;

      float thetaideg;
      thetaideg = thetai * (180 / pi);



      e_ts = millis() - kine_ts;
      if (e_ts >= KINEMATICS_UPDATE) {

        phiLeft = (dpr * (count_eLeft - prev_eLeft)) ;
        phiRight = (dpr * (count_eRight - prev_eRight));

        xr = ((phiLeft) / 2) + ((phiRight) / 2);
        thetar = ((phiLeft) / (2 * leg)) - ((phiRight) / (2 * leg));

        xi = xi + (xr * cos(thetai));
        yi = yi + (xr * sin(thetai));
        thetai = thetai + thetar;

        //    Serial.println(xr);

        prev_eLeft = count_eLeft;
        prev_eRight = count_eRight;

        kine_ts = millis();
      }
//
//      Serial.print(xi);
//      Serial.print(",");
//      Serial.print(yi);
//      Serial.print(",");
//      Serial.println(thetaideg);

    }

};



#endif
