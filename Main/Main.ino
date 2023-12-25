#include <ECE3.h>
#undef min
#undef max
#include <array>

#include "Main.hpp"
#include "Wheels.hpp"
#include "SensorArray.hpp"


float prevError = 0,
      errorIntegral = 0,
      prevCorrection = 0;
int nDonuts = 0;

float getError() {
  std::array<float, N_SENSORS> normSensorVals = normalizeSensorVals(getAvgSensorVals());

  //Sensor Fusion
  float error = 0;
  for (int i = 0; i < N_SENSORS; i++) {
    error += normSensorVals[i] * SENSOR_WEIGHTS[i];
  }
  return error;
}


void setup() {
  ECE3_Init();
  Serial.begin(115200);
  if (!initWheels() || !initSensors()) exit(1);

  //Indicate done callibrating
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  awaitBump1();
  digitalWrite(LED_PIN, LOW);

  delay(500);
}

void loop() {
  //Could be a little different because of boosting but it shouldn't make a huge difference
  int prevLSpeed = BASE_SPEED-prevCorrection,
      prevRSpeed = BASE_SPEED+prevCorrection;

  if (getSensorSum() > DONUT_ERR_THRESH) {
    donut(prevLSpeed, prevRSpeed);
    //Car reached the start
    if (nDonuts == 1) {
      updateWheelVelocities(prevLSpeed, 0, prevRSpeed, 0);
      exit(0); 
    }
    nDonuts++;
  }
  
  float error = getError();
  float errorDerivative = (error - prevError);
  errorIntegral += error;
  int correction = (int)(Kp*error + Kd*errorDerivative + Ki*errorIntegral);
  int lSpeed = BASE_SPEED,
      rSpeed = BASE_SPEED;

  if (abs(correction) < STRAIGHT_CORRECTION_THRESH) {
    //On a straightaway
    lSpeed += STRAIGHT_SPEED_BOOST;
    rSpeed += STRAIGHT_SPEED_BOOST;
    correction = (int)(correction / 4); //Reduce oscillations
  } 

  //Neg error => turn right, positive error => turn left
  lSpeed -= correction;
  rSpeed += correction;
  
  updateWheelVelocities(prevLSpeed, lSpeed, 
                        prevRSpeed, rSpeed, 2, 10);
  prevError = error;
  prevCorrection = correction;
}