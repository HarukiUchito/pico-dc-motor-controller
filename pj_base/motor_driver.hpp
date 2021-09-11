#ifndef MOTOR_DRIVER_HPP
#define MOTOR_DRIVER_HPP

#include <stdio.h>

enum MD_State {
    NO_CONTROL,
    BRAKE,
    FORWARD,
    BACK,
};

class MotorDriver {
    public:
        MotorDriver(unsigned int in1, unsigned int in2, unsigned int pwm, uint32_t pwm_max);
        void set_state(MD_State state);
        void set_duty(double);
    private:
        unsigned int mIn1, mIn2, mPWM;
        MD_State mState;
        double mDuty;
        uint32_t mPWM_MAX;
};

#endif