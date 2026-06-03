#include "pid.h"
#include <Arduino.h>

PID::PID(const double sampling_period, const double d_filter_cutoff_freq) : sampling_period(sampling_period), d_filter_cutoff_freq(d_filter_cutoff_freq)
{
    update_filter_coeffs();
}

void PID::update_filter_coeffs()
{
    const double denom = 1 + this->sampling_period * 2 * PI * this->d_filter_cutoff_freq;
    if (denom <= 0)
    {
        this->dFilterCoeff1 = 1;
        this->dFilterCoeff2 = 0;
        return;
    }
    this->dFilterCoeff1 = 1 / denom;
    this->dFilterCoeff2 = 1 - this->dFilterCoeff1;
}

double PID::compute_output(const double ref, const double feedback)
{
    const double error = ref - feedback;

    const double pTerm = this->Kp * error;

    const double iTerm = this->iStorage + this->Ki * this->sampling_period * error * this->satRecord;
    this->iStorage = iTerm;

    const double error_derivative = (error - this->previous_error) / this->sampling_period;
    const double dFiltered = this->dFilterCoeff1 * error_derivative + this->dFilterCoeff2 * this->dStorage1;
    this->dStorage1 = dFiltered;
    const double dTerm = this->Kd * dFiltered;

    this->previous_error = error;

    const double preSatOutput = pTerm + iTerm + dTerm;
    const double output = min(max(preSatOutput, this->Umin), this->Umax);
    this->satRecord = (output == preSatOutput) ? 1 : 0;

    return output;
}

void PID::clear_pid_storage()
{
    this->iStorage = 0;
    this->dStorage1 = 0;
    this->dStorage2 = 0;
    this->satRecord = 0;
    this->previous_error = 0;
}