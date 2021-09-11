#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#include "pico/stdlib.h"

const int EN_RIGHT_A = 27; const int EN_RIGHT_B = 28;
const int EN_LEFT_A = 21; const int EN_LEFT_B = 22;
const uint32_t PWM_CNT = 125 * 100 - 1;

const double gP = 40.0; const double gI = 0.5; const double gD = 0;

const int IN1_A = 6; const int IN2_A = 7; const int PWM_A = 8;
const int IN1_B = 4; const int IN2_B = 5; const int PWM_B = 3;

const int BATTERY_V_PIN = 26;
const int BATTERY_INDICATOR = 15;
const float conversion_factor = 3.3f / (1 << 12);

const double TREAD = 0.31;

const uint32_t MAIN_PERIOD = 25;

#endif