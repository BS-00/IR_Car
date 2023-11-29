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
//unsigned long lastUpdateMillis = 0;


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
  //lastUpdateMillis = millis();
  Serial.println("Done callibrating.");
  delay(5000);
  Serial.println("Starting now.");
}

enum State { STRAIGHT, DONUT };
State state = STRAIGHT;
void loop() {
  float error = getError();

  switch(state) {
  case STRAIGHT: {
    //Not necessary as it is accounted for in the prop constants
    //float dt = millis() - lastUpdateMillis; 
    float errorDerivative = (error - prevError);
    errorIntegral += error;
    int correction = (int)(Kp*error + Kd*errorDerivative + Ki*errorIntegral);
    int speedIncrease = 0;
    
    //lastUpdateMillis = millis();
    //Neg error => turn right, positive error => turn left
    updateWheelVelocities(BASE_SPEED-prevCorrection, BASE_SPEED-correction+speedIncrease, 
                          BASE_SPEED+prevCorrection, BASE_SPEED+correction+speedIncrease, 1, 0);
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
}