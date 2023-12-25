#ifndef MAIN_H
#define MAIN_H

#undef min
#undef max

const int LED_PIN = 41;

//PID SPEED CONTROL
const int BASE_SPEED = 50,
          MAX_SPEED = 250,
          STRAIGHT_SPEED_BOOST = 0;

//Proportionality constants
const float Kp = .0055,
            Ki = .000001,
            Kd = .0025;

//Determines if the car is on a curve/straightaway based on these values
const int STRAIGHT_CORRECTION_THRESH = 15,
          DONUT_ERR_THRESH = 18000;

#endif