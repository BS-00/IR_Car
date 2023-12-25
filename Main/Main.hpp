#ifndef MAIN_H
#define MAIN_H

#undef min
#undef max

const int LED_PIN = 41;

//PID SPEED CONTROL
const int BASE_SPEED = 65,
          MAX_SPEED = 250,
          CURVE_SPEED_DECREASE = 20,
          STRAIGHT_SPEED_BOOST = 10;

//Proportionality constants
const float Kp = .0043,
            Ki = .000001,
            Kd = .003;

//Determines if the car is on a curve/straightaway based on these values
const int CURVE_ERR_THRESH = 9000,
          STRAIGHT_ERR_THRESH = 3000,
          DONUT_ERR_THRESH = 18000;

#endif