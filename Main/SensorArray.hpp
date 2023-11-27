#ifndef SENSORARRAY_H
#define SENSORARRAY_H

#include <array>

const int N_SENSORS = 8;
const int SENSOR_WEIGHTS[] = {-15, -14, -12, -8, 8, 12, 14, 15};
//const float MIN_SENSOR_VALS[] = {700.6, 644.2, 649.2, 593.2, 607.6, 635.4, 682.4, 696.4}; 

const float MAX_NORM_VAL = 1000,
            MIN_NORM_VAL = 0,
            NORM_FACTORS[] = {0.55574, 0.53885, 0.54897, 0.52444, 0.52843, 0.53631, 0.55018, 0.55444};


int initSensors(std::array<float, N_SENSORS>&);
std::array<float, N_SENSORS> getAvgSensorVals(const int=5, const int=20);
#endif