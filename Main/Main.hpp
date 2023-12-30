#ifndef MAIN_H
#define MAIN_H

#undef min
#undef max

const int LED_PIN = 41;

//Speed COntrol
const int BASE_SPEED = 65,
          MAX_SPEED = 250,
          CURVE_SPEED_DECREASE = 20,
          STRAIGHT_SPEED_BOOST = 7;

//Proportionality constants (PID)
const float Kp = .00505,
            Ki = .0000001,
            Kd = .00335;

//Determines if the car is on a curve/straightaway/crosspeice based on these values
const int CURVE_ERR_THRESH = 7000,
          STRAIGHT_ERR_THRESH = 2500,
          DONUT_ERR_THRESH = 18000;

#endif