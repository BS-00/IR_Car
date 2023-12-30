#ifndef SENSORARRAY_H
#define SENSORARRAY_H

#undef min
#undef max
#include <array>


class SensorArray {
public:
    static constexpr int N_SENSORS = 8;
    static constexpr std::array<int, N_SENSORS> SENSOR_WEIGHTS = {{-15, -14, -12, -8, 8, 12, 14, 15}};
    static constexpr std::array<float, N_SENSORS> NORM_FACTORS = {{0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5}};
    static constexpr float MAX_NORM_VAL = 1000,
                           MIN_NORM_VAL = 0;

    std::array<float, N_SENSORS> minSensorVals;

    SensorArray() : _initialized(false) {}
    static int awaitBump(int);

    int init();
    int sensorSum() const;
    int error() const;
    std::array<float, N_SENSORS> avgSensorVals(const int=2, const int=5, const bool=true) const;

private:
    static constexpr int _BUMP0_PIN = 24;
    static constexpr int _DARK_THRESH = 1800, 
                         _BRIGHT_THRESH = 1300; 
    
    bool _initialized;

    std::array<float, N_SENSORS> normSensorVals() const;
    bool onLine(const std::array<float, N_SENSORS>&) const;
    std::array<bool, 2> detectObstacles(const std::array<float, N_SENSORS>&) const;
    std::array<float, N_SENSORS> normalizeSensorVals(const std::array<float, N_SENSORS>&) const;
};

#endif