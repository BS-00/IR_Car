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
unsigned long lastUpdateMillis = 0;


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
  ECE3_Init();
  Serial.begin(9600);
  if (!initWheels() || !initSensors(minSensorVals)) exit(1);
  lastUpdateMillis = millis();
}

enum State { STRAIGHT, DONUT };
State state = STRAIGHT;
void loop() {
  switch(state) {
  case STRAIGHT: {
    float error = getError();
    float dt = millis() - lastUpdateMillis;
    float errorDerivative = (error - prevError)/dt;
    errorIntegral += error * dt;
    
    float pidOut = constrain(Kp*error + Kd*errorDerivative + Ki*errorIntegral, -MAX_PID_VAL, MAX_PID_VAL);
    int correction = map(pidOut, -MAX_PID_VAL, MAX_PID_VAL, -TUNABLE_SPEED, TUNABLE_SPEED);
    lastUpdateMillis = millis();
    //Figure out which of left/right should - correction, randomly chosen atm
    updateWheelSpeeds(BASE_SPEED+prevCorrection, BASE_SPEED+correction, 
                      BASE_SPEED-prevCorrection, BASE_SPEED-correction);

    prevError = error;
    prevCorrection = correction;
    break;
  }
  case DONUT:
    donut();
    break;
  default: 
    Serial.println("ERROR: State variable not defined, exiting");
    exit(1);
    break;
  }
  delay(LOOP_DELAY_MILLIS);
}