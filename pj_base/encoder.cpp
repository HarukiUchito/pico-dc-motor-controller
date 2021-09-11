#include "encoder.hpp"

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

Encoder::Encoder(unsigned int inA, unsigned int inB) :
    mInA(inA),
    mInB(inB),
    mCnt(0.0),
    mLastCnt(0.0),
    mLastTime(0.0),
    mRPS(0.0)
{
    gpio_init(mInA); gpio_set_dir(mInA, GPIO_IN); gpio_pull_up(mInA);
    gpio_init(mInB); gpio_set_dir(mInB, GPIO_IN); gpio_pull_up(mInB);
}

void Encoder::pulse_callback()
{
    gpio_set_irq_enabled(mInA, GPIO_IRQ_EDGE_RISE, false);
    gpio_put(25, !gpio_get(25));

    if (gpio_get(mInB)) {
        mCnt++;
    } else {
        mCnt--;
    }
    gpio_set_irq_enabled(mInA, GPIO_IRQ_EDGE_RISE, true);
}

void Encoder::update()
{
    double c_cnt = mCnt, mCnt = 0;
    double ctime = double(to_us_since_boot(get_absolute_time()));
    mDt = ctime - mLastTime;

    if (mDt == 0.0)
        mRPS = 0.0;
    else
        mRPS = (c_cnt - mLastCnt) / 600.0 * (1000000.0 / mDt);

    mLastCnt = c_cnt;
    mLastTime = ctime;
}

