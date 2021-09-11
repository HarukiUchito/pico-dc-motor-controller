#include <stdio.h>
#include <cmath>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "hardware/pwm.h"
#include "hardware/resets.h"

#include "motor_driver.hpp"
#include "encoder.hpp"
#include "speed_controller.hpp"
#include "constants.hpp"

const uint LED_PIN = PICO_DEFAULT_LED_PIN;

uint8_t rbuf[256];

typedef union Float_t {
    int32_t data_i;
    float data_f;
} Float_t;

class FloatManip {
    public:
        FloatManip() {
            data_.data_f = 0.0;
        }

        float read_from_buf(uint8_t* bufptr, int idx) {
            data_.data_i = 0.0;
            data_.data_i |= ((uint32_t)bufptr[idx + 0] << 24);
            data_.data_i |= ((uint32_t)bufptr[idx + 1] << 16);
            data_.data_i |= ((uint32_t)bufptr[idx + 2] <<  8);
            data_.data_i |= ((uint32_t)bufptr[idx + 3] <<  0);
            return data_.data_f;
        }
        
        void write_to_buf(uint8_t* bufptr, int idx, float data) {
            data_.data_f = data;
            bufptr[idx + 0] = (uint8_t)((data_.data_i & 0xFF000000) >> 24);
            bufptr[idx + 1] = (uint8_t)((data_.data_i & 0x00FF0000) >> 16);
            bufptr[idx + 2] = (uint8_t)((data_.data_i & 0x0000FF00) >>  8);
            bufptr[idx + 3] = (uint8_t)((data_.data_i & 0x000000FF) >>  0);
        }
    private:
        Float_t data_;
};

FloatManip floatManip_;

Encoder en_right_(EN_RIGHT_A, EN_RIGHT_B);
Encoder en_left_(EN_LEFT_A, EN_LEFT_B);

MotorDriver md_left_(IN1_B, IN2_B, PWM_B, PWM_CNT);
MotorDriver md_right_(IN1_A, IN2_A, PWM_A, PWM_CNT);

SpeedController *spdCtrlL, *spdCtrlR;

double target_linear_vel_ = 0.0;
double target_angular_vel_ = 0.0;

uint32_t last_normal_comm_ = 0;

bool timer_callback(repeating_timer *rt)
{
    en_right_.update();
    en_left_.update();
    
    spdCtrlL->controlVelocity();
    spdCtrlR->controlVelocity();

    return true;
}

void alarm_callback(unsigned int id) {
    uint32_t ctime = to_ms_since_boot(get_absolute_time());
    if ((ctime - last_normal_comm_) > 2000) {
        printf("reset required : %u\n", ctime - last_normal_comm_);

        spdCtrlL->setVelocity(0.0);
        spdCtrlR->setVelocity(0.0);

        reset_block(RESETS_RESET_I2C0_BITS);
        unreset_block_wait(RESETS_RESET_I2C0_BITS);
        printf("reset i2c done\n");
        i2c_init(i2c0, 100000);
        i2c_set_slave_mode(i2c0, true, 0x08);

        last_normal_comm_ = to_ms_since_boot(get_absolute_time());
    }

    size_t num = i2c_get_read_available(i2c0);
    if (num > 9) {
        // calc. vehicle linear/angular velocity from wheel speed
        double c_vl = en_left_.get_rps(), c_vr = -en_right_.get_rps();
        double c_w = (c_vr - c_vl) / TREAD;
        double c_v = (c_vr + c_vl) / 2.0;

        i2c_read_raw_blocking(i2c0, rbuf, 10);
        target_linear_vel_ = floatManip_.read_from_buf(rbuf, 1);
        target_angular_vel_ = floatManip_.read_from_buf(rbuf, 5);

        floatManip_.write_to_buf(rbuf, 0, c_v);
        floatManip_.write_to_buf(rbuf, 4, c_w);
        i2c_write_raw_blocking(i2c0, rbuf, 8);

        // calc. tareget wheel speed from target linear/angular velocity
        double vr = target_linear_vel_ + TREAD * target_angular_vel_ / 2.0;
        double vl = target_linear_vel_ - TREAD * target_angular_vel_ / 2.0;

        vl = std::min(std::max(vl, -1.0), 1.0);
        vr = std::min(std::max(vr, -1.0), 1.0);
        spdCtrlL->setVelocity(vl);
        spdCtrlR->setVelocity(vr);

        last_normal_comm_ = to_ms_since_boot(get_absolute_time());
    }

    uint16_t result = adc_read();
    double voltage = result * conversion_factor * 3.0;
    //printf("Raw value: 0x%03x, voltage: %f V\n", result, voltage);
    if (voltage < 6.7)
        gpio_put(BATTERY_INDICATOR, 1);
    else
        gpio_put(BATTERY_INDICATOR, 0);
    
    hardware_alarm_set_target(id, make_timeout_time_ms(MAIN_PERIOD));
}

void encoder_callback(uint gpio, uint32_t events)
{
    if (gpio == EN_RIGHT_A)
        en_right_.pulse_callback();
    else
        en_left_.pulse_callback();
}

int main()
{
    stdio_init_all();
    adc_init();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    gpio_init(BATTERY_INDICATOR);
    gpio_set_dir(BATTERY_INDICATOR, GPIO_OUT);

    adc_gpio_init(BATTERY_V_PIN);
    adc_select_input(0);

    i2c_init(i2c0, 100000);
    i2c_set_slave_mode(i2c0, true, 0x08);
    gpio_set_function(16, GPIO_FUNC_I2C);
    gpio_set_function(17, GPIO_FUNC_I2C);
    gpio_pull_up(16);
    gpio_pull_up(17);
    // Make the I2C pins available to picotool
    //bi_decl(bi_2pins_with_func(16, 17, GPIO_FUNC_I2C));

    gpio_set_irq_enabled_with_callback(EN_RIGHT_A, GPIO_IRQ_EDGE_RISE, false, &encoder_callback);
    gpio_set_irq_enabled(EN_RIGHT_A, GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(EN_LEFT_A, GPIO_IRQ_EDGE_RISE, true);

    spdCtrlL = new SpeedController(&md_left_, &en_left_, gP, gI, gD);
    spdCtrlR = new SpeedController(&md_right_, &en_right_, gP, gI, gD);

    spdCtrlL->setVelocity(0.0);
    spdCtrlR->setVelocity(0.0);

    hardware_alarm_set_callback(2, &alarm_callback);
    hardware_alarm_set_target(2, make_timeout_time_ms(MAIN_PERIOD));

    int hz = 50;
    repeating_timer timer;
    if (!add_repeating_timer_us(-1000000 / hz, timer_callback, NULL, &timer)) {
        printf("failed to add timer");
        return 1;
    }

    while (true) {
    }

    return 0;
}