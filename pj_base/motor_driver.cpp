#include "motor_driver.hpp"

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

MotorDriver::MotorDriver(unsigned int in1, unsigned int in2, unsigned int pwm, uint32_t pwm_max) :
    mState(NO_CONTROL),
    mDuty(0.0)
{
    mIn1 = in1;
    mIn2 = in2;
    mPWM = pwm;
    mPWM_MAX = pwm_max;

    gpio_init(mIn1); gpio_set_dir(mIn1, GPIO_OUT);
    gpio_init(mIn2); gpio_set_dir(mIn2, GPIO_OUT);

    gpio_set_function(mPWM, GPIO_FUNC_PWM);
    unsigned int slice_num = pwm_gpio_to_slice_num(mPWM);
    pwm_set_wrap(slice_num, mPWM_MAX);
    pwm_set_gpio_level(mPWM, 0);
    pwm_set_enabled(slice_num, true);
}

void MotorDriver::set_state(MD_State state)
{
    if (mState == state)
        return;
    mState = state;
    switch (mState)
    {
    case NO_CONTROL:
        gpio_put(mIn1, 0);
        gpio_put(mIn2, 0);
        break;
    case BRAKE:
        gpio_put(mIn1, 1);
        gpio_put(mIn2, 1);
        break;
    case FORWARD:
        gpio_put(mIn1, 1);
        gpio_put(mIn2, 0);
        break;
    case BACK:
        gpio_put(mIn1, 0);
        gpio_put(mIn2, 1);
        break;
    default:
        break;
    }
}

void MotorDriver::set_duty(double duty)
{
    mDuty = duty;
    if (mState == NO_CONTROL || mState == BRAKE)
        mDuty = 0.0;
    pwm_set_gpio_level(mPWM, mDuty * mPWM_MAX / 100.0);
}
