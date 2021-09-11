#ifndef SPEED_CONTROLLER_HPP
#define SPEED_CONTROLLER_HPP

#include "motor_driver.hpp"
#include "encoder.hpp"

class SpeedController {
    public:
        SpeedController(
            MotorDriver* md_ptr,
            Encoder* enc_ptr,
            double gainP, double gainI, double gainD);

        void setMotorDuty(double duty);
        void controlVelocity();

        void setVelocity(double velocity);

        double get_rps() const { return mRPS; };
        double get_velocity() const { return currentVelocity_; };
    private:
        double mP, mI, mD; // PID gain
        double mIntegralSum, mRPS, targetVelocity_, currentVelocity_;
        MotorDriver* mMD;
        Encoder* mEncoder;
};

#endif