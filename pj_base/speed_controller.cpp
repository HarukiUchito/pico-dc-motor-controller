#include "speed_controller.hpp"
#include <cmath>

SpeedController::SpeedController(
    MotorDriver* md_ptr,
    Encoder* enc_ptr,
    double gainP, double gainI, double gainD
) : mMD(md_ptr), mEncoder(enc_ptr),
    mP(gainP), mI(gainI), mD(gainD),
    mIntegralSum(0.0), mRPS(0.0),
    targetVelocity_(0.0), currentVelocity_(0.0)
{

}

void SpeedController::setMotorDuty(double duty)
{
    if (duty < 0.0)
        mMD->set_state(MD_State::BACK);
    else if (duty > 0.0)
        mMD->set_state(MD_State::FORWARD);
    else
        mMD->set_state(MD_State::BRAKE);
    
    mMD->set_duty(std::abs(duty));
}

void SpeedController::controlVelocity()
{
    double lastRPS = mRPS;
    mRPS = std::abs(mEncoder->get_rps());
    double dt = mEncoder->get_dt() / 1000.0;

    // ramp acceleration
    double max_acc = 0.02; // r/s^2
    double e_vel = targetVelocity_ - currentVelocity_;
    double ctrl = (e_vel > 0.0) ? 
        std::min(max_acc * dt, e_vel) : // in case of acceleration
        std::max(-max_acc * dt, e_vel); // deacceleration
    currentVelocity_ += ctrl;

    // speed PID control
//currentVelocity_  = targetVelocity_;
    double e_rps = std::abs(currentVelocity_) - mRPS;

//printf("target: %f\n", currentVelocity_);
//printf("e: %f\n", e_rps);

    double nduty = 0.0;
    nduty += mP * e_rps;

    mIntegralSum += e_rps * dt;
    nduty += mI * mIntegralSum;

    if (dt == 0.0)
        dt = 1;
    nduty += mD * (mRPS - lastRPS) / dt;

    nduty = std::min(nduty, 100.0);
    nduty = std::max(nduty, 0.0);

//printf("duty: %f\n", nduty);

    // switch direction
    if (currentVelocity_ < 0.0)
        mMD->set_state(MD_State::BACK);
    else if (currentVelocity_ > 0.0)
        mMD->set_state(MD_State::FORWARD);
    else
        mMD->set_state(MD_State::BRAKE);
    
    mMD->set_duty(nduty);
}

void SpeedController::setVelocity(double velocity)
{
    targetVelocity_ = velocity;
}
