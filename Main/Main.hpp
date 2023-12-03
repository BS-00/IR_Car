#ifndef MAIN_H
#define MAIN_H

//PID SPEED CONTROL
const int BASE_SPEED = 70, 
          MAX_SPEED = 220,
          STRAIGHT_SPEED_BOOST = 80,
          CURVE_SPEED_DECREASE = 25;

//Proportionality constants
const float Kp = .0037,
            Ki = .000001,
            Kd = .011;

//Determines if the car is on a curve/straightaway based on these values
const int STRAIGHT_CORRECTION_THRESH = 20,
          CURVE_CORRECTION_THRESH = 55,
          DONUT_ERR_THRESH = 20000;

#endif