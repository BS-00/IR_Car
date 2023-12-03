#include <ECE3.h>
#undef min
#undef max
#include <array>
#include "Main.hpp"
#include "Wheels.hpp"
#include "SensorArray.hpp"


std::array<float, N_SENSORS> minSensorVals;

float prevError = 0,
      errorIntegral = 0,
      prevCorrection = 0;
    
int nDonuts = 0;


float getError() {
  std::array<float, N_SENSORS> avgSensorVals = getAvgSensorVals();
  float error = 0;
  for (int i = 0; i < N_SENSORS; i++) {
    //Normalize sensor values (some cuttoff can occur here but hopefully it will not make a large difference)
    float normSensorVal = std::min(std::max((avgSensorVals[i] - minSensorVals[i]) * NORM_FACTORS[i], MIN_NORM_VAL), MAX_NORM_VAL);
    //Calaulate sensor fusion error
    error += normSensorVal * SENSOR_WEIGHTS[i];
  }
  return error;
}


void setup() {
  delay(3000);
  ECE3_Init();
  Serial.begin(115200);
  Serial.println("Callibrating...");
  if (!initWheels() || !initSensors(minSensorVals)) exit(1);
  Serial.println("Done callibrating.");
  delay(5000);
  Serial.println("Starting now.");
}

void loop() {
  float error = getError();
  
  if (abs(error) > DONUT_ERR_THRESH) {
    if (nDonuts == 1) exit(0); //Car reached the start
    error = getError();
  }

  float errorDerivative = (error - prevError);
  errorIntegral += error;
  int correction = (int)(Kp*error + Kd*errorDerivative + Ki*errorIntegral);
  int lSpeed = BASE_SPEED,
      rSpeed = BASE_SPEED;

  if (abs(correction) < STRAIGHT_CORRECTION_THRESH) {
    //On a straightaway
    lSpeed += STRAIGHT_SPEED_BOOST;
    rSpeed += STRAIGHT_SPEED_BOOST;
    correction = (int)(correction / 5); //Reduce oscillations
  } else if (abs(correction) > CURVE_CORRECTION_THRESH) {
    //On a curve
    lSpeed -= CURVE_SPEED_DECREASE;
    rSpeed -= CURVE_SPEED_DECREASE;
  }

  //Neg error => turn right, positive error => turn left
  lSpeed -= correction;
  rSpeed += correction;
  
  updateWheelVelocities(BASE_SPEED-prevCorrection, lSpeed, 
                        BASE_SPEED+prevCorrection, rSpeed, 2, 10);
  prevError = error;
  prevCorrection = correction;
}