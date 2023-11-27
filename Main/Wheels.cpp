#include "Wheels.hpp"
#include <ECE3.h>

int initWheels() {
    pinMode(L_NSLP_PIN, OUTPUT);
    pinMode(L_DIR_PIN, OUTPUT);
    pinMode(L_PWM_PIN, OUTPUT);
    pinMode(R_NSLP_PIN, OUTPUT);
    pinMode(R_DIR_PIN, OUTPUT);
    pinMode(R_PWM_PIN, OUTPUT); 

    resetEncoderCount_left();
    resetEncoderCount_right();

    //Enable motors
    digitalWrite(L_NSLP_PIN, HIGH);
    digitalWrite(R_NSLP_PIN, HIGH);

    return 1;
}

int updateWheelSpeeds(const int initRSpeed, const int finalRSpeed, 
                      const int initLSpeed,  const int finalLSpeed, 
                      const int nSteps, const int delayMillis) {
  int lPWM = initLSpeed, rPWM = initRSpeed;
  const int dL = (finalLSpeed - initLSpeed)/nSteps,
            dR = (finalRSpeed - initRSpeed)/nSteps;
  for (int i = 0; i < nSteps; i++) {
    lPWM += dL; rPWM += dR;
    if (initLSpeed > 0 && finalLSpeed > 0) analogWrite(L_PWM_PIN, lPWM);
    if (initRSpeed > 0 && finalRSpeed > 0) analogWrite(R_PWM_PIN, rPWM);
    delay(delayMillis);
  }
  return 1;
}

//Turn the car 180 deg
int donut() {
  
  return 1;
}