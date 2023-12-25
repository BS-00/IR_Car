#ifndef MAIN_H
#define MAIN_H

#undef min
#undef max

const int LED_PIN = 41;

//PID SPEED CONTROL
const int BASE_SPEED = 65,
          MAX_SPEED = 250,
          CURVE_SPEED_DECREASE = 20,
          STRAIGHT_SPEED_BOOST = 5;

//Proportionality constants
const float Kp = .00525,
            Ki = .0000001,
            Kd = .0032;

//Determines if the car is on a curve/straightaway based on these values
const int CURVE_ERR_THRESH = 8000,
          STRAIGHT_ERR_THRESH = 2500,
          DONUT_ERR_THRESH = 18000;

#endif