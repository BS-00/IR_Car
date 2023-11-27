#ifndef WHEELS_H
#define WHEELS_H

//PINS
const int L_NSLP_PIN = 31, 
          L_DIR_PIN = 29,
          L_PWM_PIN = 40;
const int R_NSLP_PIN = 11, 
          R_DIR_PIN = 30,
          R_PWM_PIN = 39;

int initWheels();
int updateWheelSpeeds(const int=-1, const int=-1, const int=-1,  
                      const int=-1, const int=5, const int=50);
int donut();

#endif