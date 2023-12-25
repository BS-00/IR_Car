#ifndef SENSORARRAY_H
#define SENSORARRAY_H

#undef min
#undef max
#include <array>

const int N_SENSORS = 8;
const int SENSOR_WEIGHTS[] = {-15, -14, -12, -8, 8, 12, 14, 15};

const float MAX_NORM_VAL = 1000,
            MIN_NORM_VAL = 0,
            NORM_FACTORS[] = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};

const int BUMP1_PIN = 24;

const int DARK_THRESHOLD = 1600,
          BRIGHT_BUFFER = 200;


static std::array<float, N_SENSORS> minSensorVals;


int initSensors();
int getSensorSum();
std::array<float, N_SENSORS> getAvgSensorVals(const int=2, const int=5, const bool=true);
std::array<float, N_SENSORS> normalizeSensorVals(const std::array<float, N_SENSORS>&);
int awaitBump1();

#endif