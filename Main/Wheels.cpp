#include <ECE3.h>
#undef min
#undef max

#include "Wheels.hpp"
#include "Main.hpp"

Wheels::Wheels() {}
int Wheels::init() {
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

int Wheels::updateWheelVelocities(const int initLVelocity,  const int finalLVelocity, 
                                  const int initRVelocity, const int finalRVelocity, 
                                  const int nSteps, const int delayMillis) const {
  constrain(initLVelocity, -MAX_SPEED, MAX_SPEED);
  constrain(finalLVelocity, -MAX_SPEED, MAX_SPEED);
  constrain(initRVelocity, -MAX_SPEED, MAX_SPEED);
  constrain(finalRVelocity, -MAX_SPEED, MAX_SPEED);

  int lPWM = initLVelocity, rPWM = initRVelocity;
  const int dL = (int)((finalLVelocity - initLVelocity)/nSteps),
            dR = (int)((finalRVelocity - initRVelocity)/nSteps);
  for (int i = 0; i < nSteps; i++) {
    lPWM += dL; rPWM += dR;
    //Set wheel directions
    digitalWrite(L_DIR_PIN, lPWM < 0);
    digitalWrite(R_DIR_PIN, rPWM < 0);

    analogWrite(L_PWM_PIN, abs(lPWM));
    analogWrite(R_PWM_PIN, abs(rPWM));
    delay(delayMillis);
  }
  return 1;
}

//Turn the car ~180 deg
int Wheels::donut(const int initLVelocity, const int initRVelocity) const {
  updateWheelVelocities(initLVelocity, DONUT_SPEED, initRVelocity, -DONUT_SPEED);
  delay(DONUT_DELAY_MS);
  updateWheelVelocities(DONUT_SPEED, BASE_SPEED, -DONUT_SPEED, BASE_SPEED, 2);
  delay(DONUT_FWD_DELAY_MS);
  return 1;
}