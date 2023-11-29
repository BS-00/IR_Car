#ifndef SENSORARRAY_H
#define SENSORARRAY_H

#include <array>

const int N_SENSORS = 8;
const int SENSOR_WEIGHTS[] = {-15, -14, -12, -8, 8, 12, 14, 15};

const float MAX_NORM_VAL = 1000,
            MIN_NORM_VAL = 0,
            NORM_FACTORS[] = {0.55, 0.55, 0.55, 0.55, 0.55, 0.55, 0.55, 0.55};


int initSensors(std::array<float, N_SENSORS>&);
std::array<float, N_SENSORS> getAvgSensorVals(const int=2, const int=10);

#endif