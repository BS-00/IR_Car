#include <ECE3.h>
#undef min
#undef max
#include <array>

#include "SensorArray.hpp"
#include "Main.hpp"


//Private helper functions
//Checks if any of the middle sensors are dark
bool onLine(const std::array<float, N_SENSORS>& sensorVals) {
  float midSensorIndex = N_SENSORS/2;
  if ((int)midSensorIndex == midSensorIndex && midSensorIndex+1 < N_SENSORS) {
    return sensorVals[midSensorIndex-1] >= minSensorVals[midSensorIndex-1]+BRIGHT_BUFFER || 
           sensorVals[midSensorIndex] >= minSensorVals[midSensorIndex]+BRIGHT_BUFFER ||
           sensorVals[midSensorIndex-2] >= minSensorVals[midSensorIndex-2]+BRIGHT_BUFFER || 
           sensorVals[midSensorIndex+1] >= minSensorVals[midSensorIndex+1]+BRIGHT_BUFFER;
  }
  //Car does not have enough sensors or has an odd number of sensors
  exit(1);
}

//Returns a boolean array containing if an obstacle was detected on each side of the car [left, right]
std::array<bool, 2> detectObstacles(const std::array<float, N_SENSORS>& sensorVals) {
  if (!onLine(sensorVals)) return {{0, 0}};
  return {{sensorVals[0] > DARK_THRESHOLD, sensorVals[N_SENSORS-1] > DARK_THRESHOLD}};
}

std::array<float, N_SENSORS> normalizeSensorVals(const std::array<float, N_SENSORS>& sensorVals) {
  std::array<float, N_SENSORS> normSensorVals;
  for (int i = 0; i < N_SENSORS; i++) {
    float normVal = (sensorVals[i] - minSensorVals[i]) * NORM_FACTORS[i];
    normSensorVals[i] = constrain(normVal, MIN_NORM_VAL, MAX_NORM_VAL);
  }
  return normSensorVals;
}


//Public sensor array functions
int initSensors() {
  pinMode(BUMP1_PIN, INPUT_PULLUP);

  minSensorVals = getAvgSensorVals(10, 20);
  for (int i = 0; i < N_SENSORS; i++) {
      if (minSensorVals[i] > 900) {
          Serial.println("Minimum sensor values are too high, likely the car was not callibrated properly: ");
          for (int sensorVal : minSensorVals) {
              Serial.print(sensorVal);
              Serial.print("\t");
          }
          return 0;
      }
  }
  return 1;
}

std::array<float, N_SENSORS> getAvgSensorVals(const int nSamples, const int delayMillis, const bool removeOuterSensors) {
  std::array<float, N_SENSORS> avgSensorVals = {};
  //Average sensor values
  for (int i = 0; i < nSamples; i++) {
    uint16_t rawSensorVals[N_SENSORS] = {};
    ECE3_read_IR(rawSensorVals);

    for (int j = 0; j < N_SENSORS; j++) {
      avgSensorVals[j] += (float)rawSensorVals[j];
    }
    delay(delayMillis);
  }
  for (int i = 0; i < N_SENSORS; i++) {
    avgSensorVals[i] /= (float)nSamples;
  }

  //Disable outmost sensors when between obstacles
  if (removeOuterSensors) {
    std::array<bool, 2> detectedObstacles = detectObstacles(avgSensorVals);
    if (detectedObstacles[0]) {
      avgSensorVals[0] = minSensorVals[0];
      avgSensorVals[1] = minSensorVals[1];
    }
    if (detectedObstacles[1]) {
      avgSensorVals[N_SENSORS-1] = minSensorVals[N_SENSORS-1];
      avgSensorVals[N_SENSORS-2] = minSensorVals[N_SENSORS-2];
    }
  }
  return avgSensorVals;
}

int getSensorSum() {
  float sum = 0;
  std::array<float, N_SENSORS> avgSensorVals = getAvgSensorVals(1, 0, false);
  for (float sensorVal : avgSensorVals) sum += sensorVal;
  return sum;
}

int awaitBump1() {
  while (digitalRead(BUMP1_PIN)) delay(10);
  return 1;
}