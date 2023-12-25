#ifndef WHEELS_H
#define WHEELS_H

#undef min
#undef max

class Wheels {
public:
    Wheels();

    int init();
    int updateWheelVelocities(const int=-1, const int=-1, const int=-1,  
                              const int=-1, const int=4, const int=10) const;
    int donut(const int, const int) const;

private:
    static constexpr int L_NSLP_PIN = 31, 
                        L_DIR_PIN = 29,
                        L_PWM_PIN = 40,
                        R_NSLP_PIN = 11, 
                        R_DIR_PIN = 30,
                        R_PWM_PIN = 39;
    static constexpr int DONUT_SPEED = 210,
                         DONUT_DELAY_MS = 275,
                         DONUT_FWD_DELAY_MS = 220;
};

#endif