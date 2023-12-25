#include <ECE3.h>
#undef min
#undef max
#include <array>

#include "SensorArray.hpp"
#include "Main.hpp"


//Required definitions until C++17
constexpr std::array<int, SensorArray::N_SENSORS> SensorArray::SENSOR_WEIGHTS;
constexpr std::array<float, SensorArray::N_SENSORS> SensorArray::NORM_FACTORS;


SensorArray::SensorArray() : _initialized(false) {}

int SensorArray::init() {
  pinMode(_BUMP0_PIN, INPUT_PULLUP);

  minSensorVals = avgSensorVals(10, 25);

  //Ensure callibrated properly
  for (float minSensorVal : minSensorVals) {
    if (minSensorVal > _BRIGHT_THRESH) return 0;
  }
  _initialized = true;
  return 1;
}

int SensorArray::awaitBump(int switchIndex) {
  int switchPin;
  switch(switchIndex) {
    case 0 : switchPin = _BUMP0_PIN; break;
    default: return 0; break;
  }
  while (digitalRead(switchPin)) delay(10);

  return 1;
}


std::array<float, SensorArray::N_SENSORS> SensorArray::avgSensorVals(const int nSamples, const int delayMillis, const bool removeOuterSensors) const {
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
    //Serial.println("ON LINE: " + String(onLine(avgSensorVals)) + "\tLEFT DETECTED: " + String(detectedObstacles[0]) + "\tRIGHT DETECTED: " + String(detectedObstacles[1]));
    digitalWrite(LED_PIN, LOW);
    if (detectedObstacles[0]) {
      avgSensorVals[0] = minSensorVals[0];
      avgSensorVals[1] = minSensorVals[1];
      digitalWrite(LED_PIN, HIGH);
    } else {
      digitalWrite(LED_PIN, LOW);
    }
    if (detectedObstacles[1]) {
      avgSensorVals[N_SENSORS-2] = minSensorVals[N_SENSORS-2];
      avgSensorVals[N_SENSORS-1] = minSensorVals[N_SENSORS-1];
      digitalWrite(LED_PIN, HIGH);
    }
  }
  return avgSensorVals;
}

int SensorArray::error() const {
  std::array<float, N_SENSORS> _normSensorVals = normSensorVals();

  //Sensor Fusion
  float error = 0;
  for (int i = 0; i < N_SENSORS; i++) {
    error += _normSensorVals[i] * SENSOR_WEIGHTS[i];
  }
  return error;
}

int SensorArray::sensorSum() const {
  float sum = 0;
  for (float sensorVal : avgSensorVals(1, 0, false)) sum += sensorVal;
  return sum;
}


//Helper Functions

//Checks if any of the middle sensors are not bright
bool SensorArray::onLine(const std::array<float, N_SENSORS>& sensorVals) const {
  float rightMidSensorIndex = N_SENSORS/2;
  const int BUFFER = 300;
  if ((int)rightMidSensorIndex == rightMidSensorIndex && rightMidSensorIndex+1 < N_SENSORS) {
    return sensorVals[rightMidSensorIndex-2] > _BRIGHT_THRESH-BUFFER || 
           sensorVals[rightMidSensorIndex-1] > _BRIGHT_THRESH || 
           sensorVals[rightMidSensorIndex]   > _BRIGHT_THRESH ||
           sensorVals[rightMidSensorIndex+1] > _BRIGHT_THRESH-BUFFER;
  }
  //Car does not have enough sensors or has an odd number of sensors
  exit(1);
}

//Returns a boolean array containing if an obstacle was detected on each side of the car [left, right]
std::array<bool, 2> SensorArray::detectObstacles(const std::array<float, N_SENSORS>& sensorVals) const {
  if (!onLine(sensorVals)) return {{0, 0}};

  return {{sensorVals[0] > _DARK_THRESH && sensorVals[2] < _BRIGHT_THRESH, 
           sensorVals[N_SENSORS-1] > _DARK_THRESH && sensorVals[N_SENSORS-3] < _BRIGHT_THRESH}};
}

std::array<float, SensorArray::N_SENSORS> SensorArray::normSensorVals() const {
  std::array<float, N_SENSORS> sensorVals = avgSensorVals(),
                               normSensorVals;
  for (int i = 0; i < N_SENSORS; i++) {
    float normVal = (sensorVals[i] - minSensorVals[i]) * NORM_FACTORS[i];
    normSensorVals[i] = constrain(normVal, MIN_NORM_VAL, MAX_NORM_VAL);
  }
  return normSensorVals;
}