#include <ECE3.h>
#undef min
#undef max

#include "Wheels.hpp"
#include "Main.hpp"


int Wheels::init() {
    pinMode(_L_NSLP_PIN, OUTPUT);
    pinMode(_L_DIR_PIN, OUTPUT);
    pinMode(_L_PWM_PIN, OUTPUT);
    pinMode(_R_NSLP_PIN, OUTPUT);
    pinMode(_R_DIR_PIN, OUTPUT);
    pinMode(_R_PWM_PIN, OUTPUT); 

    resetEncoderCount_left();
    resetEncoderCount_right();

    //Enable motors
    digitalWrite(_L_NSLP_PIN, HIGH);
    digitalWrite(_R_NSLP_PIN, HIGH);

    return 1;
}

int Wheels::setVelocities(const int finalLVelocity, const int finalRVelocity, 
                                  const int nSteps, const int delayMillis) {
  constrain(_lVelocity, -MAX_SPEED, MAX_SPEED);
  constrain(finalLVelocity, -MAX_SPEED, MAX_SPEED);
  constrain(_rVelocity, -MAX_SPEED, MAX_SPEED);
  constrain(finalRVelocity, -MAX_SPEED, MAX_SPEED);

  int lPWM = _lVelocity, rPWM = _rVelocity;
  const int dL = (int)((finalLVelocity - _lVelocity)/nSteps),
            dR = (int)((finalRVelocity - _rVelocity)/nSteps);
  for (int i = 0; i < nSteps; i++) {
    lPWM += dL; rPWM += dR;
    //Set wheel directions
    digitalWrite(_L_DIR_PIN, lPWM < 0);
    digitalWrite(_R_DIR_PIN, rPWM < 0);

    analogWrite(_L_PWM_PIN, abs(lPWM));
    analogWrite(_R_PWM_PIN, abs(rPWM));
    delay(delayMillis);
  }
  return 1;
}

//Turn the car ~180 deg
int Wheels::donut() {
  setVelocities(_DONUT_SPEED, -_DONUT_SPEED);
  delay(_DONUT_DELAY_MS);
  setVelocities(BASE_SPEED, BASE_SPEED, 2);
  delay(_DONUT_FWD_DELAY_MS);
  return 1;
}