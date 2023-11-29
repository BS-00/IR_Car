#ifndef MAIN_H
#define MAIN_H

//PID SPEED CONTROL
const int BASE_SPEED = 65, 
          MAX_SPEED = 200;

//Proportionality constants
const float Kp = .0037,
            Ki = .000001,
            Kd = .011;

#endif