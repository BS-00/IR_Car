#ifndef MAIN_H
#define MAIN_H

const int LOOP_DELAY_MILLIS = 25;
//PID SPEED CONTROL
const int BASE_SPEED = 100,
          MIN_SPEED = 10,
          MAX_SPEED = 255;
#define TUNABLE_SPEED std::min(BASE_SPEED-MIN_SPEED, MAX_SPEED-BASE_SPEED)

//Completley arbitrary / based on prop constants?
const int MAX_PID_VAL = 7000;
//Proportionality constants (need tweaking)
const float Kp = 1,
            Ki = 1,
            Kd = 1;

#endif