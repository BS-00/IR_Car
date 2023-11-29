#include "SensorArray.hpp"
#include <ECE3.h>
#include <array>

int initSensors(std::array<float, N_SENSORS>& minSensorVals) {
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

std::array<float, N_SENSORS> getAvgSensorVals(const int nSamples, const int delayMillis) {
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

  return avgSensorVals;
}