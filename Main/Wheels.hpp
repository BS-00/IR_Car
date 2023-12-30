#ifndef WHEELS_H
#define WHEELS_H

#undef min
#undef max

class Wheels {
public:
    Wheels() : _lVelocity(0), _rVelocity(0) {}

    int init();
    int setVelocities(const int=-1, const int=-1, 
                      const int=4, const int=10);
    int halt() { return setVelocities(0, 0); };
    int donut();

private:
    static constexpr int _L_NSLP_PIN = 31, 
                         _L_DIR_PIN = 29,
                         _L_PWM_PIN = 40,
                         _R_NSLP_PIN = 11, 
                         _R_DIR_PIN = 30,
                         _R_PWM_PIN = 39;
    static constexpr int _DONUT_SPEED = 210,
                         _DONUT_DELAY_MS = 275,
                         _DONUT_FWD_DELAY_MS = 220;

    int _lVelocity, _rVelocity;
};

#endif