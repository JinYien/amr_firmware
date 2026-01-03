#ifndef PID_H
#define PID_H

#include <Arduino.h>


class PID {
public:
    explicit PID(double sampling_period = 1E-3, double d_filter_cutoff_freq = 30);
    double Kp = 1;
    double Ki = 0;
    double Kd = 0;
    double Umax = 9999;
    double Umin = -9999;

    double compute_output(double ref, double feedback);
    void clear_pid_storage();

private:
    double sampling_period = 0;
    double d_filter_cutoff_freq = 0;
    double iStorage = 0;
    double dStorage1 = 0;
    double dStorage2 = 0;
    double dFilterCoeff1 = 1;
    double dFilterCoeff2 = 0;
    double satRecord = 1;
    void update_filter_coeffs();
};

#endif // PID_H
