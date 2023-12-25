#ifndef WHEELS_H
#define WHEELS_H

#undef min
#undef max

//PINS
const int L_NSLP_PIN = 31, 
          L_DIR_PIN = 29,
          L_PWM_PIN = 40;
const int R_NSLP_PIN = 11, 
          R_DIR_PIN = 30,
          R_PWM_PIN = 39;

const int DONUT_SPEED = 220;
const int DONUT_DELAY_MS = 275,
          DONUT_FWD_DELAY_MS = 200;

int initWheels();
int updateWheelVelocities(const int=-1, const int=-1, const int=-1,  
                          const int=-1, const int=4, const int=10);
int donut(const int, const int);

#endif