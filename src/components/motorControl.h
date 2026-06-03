#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include "defs.h"
#include "pid.h"
#include "serialManager.h"

enum class CONTROL_MODE
{
    TORQUE_CONTROL = 0,
    SPEED_CONTROL = 1,
    POSITION_CONTROL = 2,
    IMPEDANCE_CONTROL = 3,
    TIMED_PULSE_CONTROL = 4,
};

class motorControl
{
public:
    explicit motorControl(Side side);
    void update(double angle_deg);
    void start_timer();
    double get_torque_command_Nm() const;
    double get_angle_deg() const;
    double get_speed_deg_per_sec() const;
    double get_acceleration_deg_per_sec2() const;
    void change_params(char id, int64_t val_i, double val_f, serialManager *ser);

    CONTROL_MODE control_mode = CONTROL_MODE::SPEED_CONTROL;

    double target_angle_deg = 0;
    double target_speed_deg_per_sec = 0;
    double target_torque_Nm = 0;
    double max_torque_Nm = 30;

    // 速度制御
    PID speed_pid;

    // 位置制御
    PID position_pid;

    // インピーダンス制御
    double stiffness = 1;
    double damping = 0;
    double inertia = 0;

    // パルス幅制御
    double timed_pulse_start_time = 0;
    double timed_pulse_end_time = 0;

private:
    char side_str[14];
    Side side;
    double angle_deg = 0;
    double speed_deg_per_sec = 0;
    double acceleration_deg_per_sec2 = 0;
    double torque_command_Nm = 0;

    void calculate_speed();
    double previous_angle_deg = 0;
    // 1次離散ローパスフィルター：filtered = a1*prev + b0*new、カットオフ周波数に調整する
    const double speed_filter_a1 = 1 / (1 + SAMPLING_PERIOD_SEC * 2 * PI * SPEED_FILTER_CUTOFF_FREQUENCY),
                 speed_filter_b0 = 1 - 1 / (1 + SAMPLING_PERIOD_SEC * 2 * PI * SPEED_FILTER_CUTOFF_FREQUENCY);

    double previous_speed_deg_per_sec = 0;
    const double acceleration_filter_a1 = 1 / (1 + SAMPLING_PERIOD_SEC * 2 * PI * ACCELERATION_FILTER_CUTOFF_FREQUENCY),
                 acceleration_filter_b0 =
                     1 - 1 / (1 + SAMPLING_PERIOD_SEC * 2 * PI * ACCELERATION_FILTER_CUTOFF_FREQUENCY);
    void calculate_acceleration();

    unsigned long timer_start_usec = 0;
};

#endif // MOTOR_CONTROL_H