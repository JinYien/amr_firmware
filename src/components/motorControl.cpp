#include "motorControl.h"

motorControl::motorControl(Side side)
    : speed_pid(SAMPLING_PERIOD_SEC, SPEED_FILTER_CUTOFF_FREQUENCY),
      position_pid(SAMPLING_PERIOD_SEC, ACCELERATION_FILTER_CUTOFF_FREQUENCY),
      side(side)
{
    this->speed_pid.Umax = 10;
    this->speed_pid.Umin = -10;
    this->position_pid.Umax = 1000;
    this->position_pid.Umin = -1000;
    if (side == Side::RIGHT)
    {
        strcpy(this->side_str, "Right Motor: ");
    }
    else if (side == Side::LEFT)
    {
        strcpy(this->side_str, "Left Motor: ");
    }
}

void motorControl::update(const double angle_deg)
{
    this->angle_deg = angle_deg;
    calculate_speed();
    calculate_acceleration();

    switch (this->control_mode)
    {
    case CONTROL_MODE::TORQUE_CONTROL:
        this->torque_command_Nm = this->target_torque_Nm;
        break;
    case CONTROL_MODE::SPEED_CONTROL:
        this->torque_command_Nm =
            this->speed_pid.compute_output(this->target_speed_deg_per_sec, this->speed_deg_per_sec);
        break;
    case CONTROL_MODE::POSITION_CONTROL:
    {
        // カスケードループ：外側の位置PIDが、内側の速度PIDに対して速度目標値を生成する
        const double target_speed = this->position_pid.compute_output(this->target_angle_deg, this->angle_deg);
        this->torque_command_Nm = this->speed_pid.compute_output(target_speed, this->speed_deg_per_sec);
        break;
    }
    case CONTROL_MODE::IMPEDANCE_CONTROL:
    {
        const double pos_error = this->target_angle_deg - this->angle_deg;
        this->torque_command_Nm = this->stiffness * pos_error + this->damping * this->speed_deg_per_sec +
                                  this->inertia * this->acceleration_deg_per_sec2;
        break;
    }
    case CONTROL_MODE::TIMED_PULSE_CONTROL:
    {
        // start_timer()の実行後、[start, end]の範囲内でのみ目標トルクを適用する
        if (double const timer_time = static_cast<double>(micros() - this->timer_start_usec) / 1E6;
            timer_time > this->timed_pulse_start_time and timer_time < this->timed_pulse_end_time)
        {
            this->torque_command_Nm = this->target_torque_Nm;
        }
        else
        {
            this->torque_command_Nm = 0;
        }
        break;
    }
    default:
        this->torque_command_Nm = 0;
        break;
    }

    // トルクを限界まで引き出す
    if (this->torque_command_Nm > this->max_torque_Nm)
    {
        this->torque_command_Nm = this->max_torque_Nm;
    }
    else if (this->torque_command_Nm < -this->max_torque_Nm)
    {
        this->torque_command_Nm = -this->max_torque_Nm;
    }
}

void motorControl::calculate_speed()
{
    const double new_speed = (this->angle_deg - this->previous_angle_deg) / SAMPLING_PERIOD_SEC;
    this->speed_deg_per_sec = this->speed_filter_a1 * this->speed_deg_per_sec + this->speed_filter_b0 * new_speed;
    this->previous_angle_deg = this->angle_deg;
}

void motorControl::calculate_acceleration()
{
    const double new_acceleration = (this->speed_deg_per_sec - this->previous_speed_deg_per_sec) / SAMPLING_PERIOD_SEC;
    this->acceleration_deg_per_sec2 = this->acceleration_filter_a1 * this->acceleration_deg_per_sec2 + this->acceleration_filter_b0 * new_acceleration;
    this->previous_speed_deg_per_sec = this->speed_deg_per_sec;
}

void motorControl::start_timer() { this->timer_start_usec = micros(); }
double motorControl::get_torque_command_Nm() const { return this->torque_command_Nm; }
double motorControl::get_angle_deg() const { return this->angle_deg; }
double motorControl::get_speed_deg_per_sec() const { return this->speed_deg_per_sec; }
double motorControl::get_acceleration_deg_per_sec2() const { return this->acceleration_deg_per_sec2; }

void motorControl::change_params(char const id, int64_t const val_i, const double val_f, serialManager *ser)
{
    char str[TX_BUFFER_SIZE];
    strcpy(str, this->side_str);
    switch (id)
    {
    case 0x11:
        this->control_mode = static_cast<CONTROL_MODE>(val_i);
        strcat(str, "Control mode set to %d.");
        ser->send_msg(str, val_i);
        break;
    case 0x21:
        this->max_torque_Nm = val_f;
        this->speed_pid.Umax = val_f;
        this->speed_pid.Umin = -val_f;
        strcat(str, "Max torque set to %f.");
        ser->send_msg(str, val_f);
        break;
    case 0x31:
        this->target_torque_Nm = val_f;
        strcat(str, "Target torque set to %f.");
        ser->send_msg(str, val_f);
        break;
    case 0x41:
        this->target_speed_deg_per_sec = val_f;
        strcat(str, "Target speed set to %f.");
        ser->send_msg(str, val_f);
        break;
    case 0x42:
        this->speed_pid.Kp = val_f;
        this->speed_pid.clear_pid_storage();
        strcat(str, "Speed P gain set to %f.");
        ser->send_msg(str, val_f);
        break;
    case 0x43:
        this->speed_pid.Ki = val_f;
        this->speed_pid.clear_pid_storage();
        strcat(str, "Speed I gain set to %f.");
        ser->send_msg(str, val_f);
        break;
    case 0x44:
        this->speed_pid.Kd = val_f;
        this->speed_pid.clear_pid_storage();
        strcat(str, "Speed D gain set to %f.");
        ser->send_msg(str, val_f);
        break;
    case 0x51:
        this->target_angle_deg = val_f;
        strcat(str, "Target angle set to %f.");
        ser->send_msg(str, val_f);
        break;
    case 0x52:
        this->position_pid.Umax = val_f;
        this->position_pid.Umin = -val_f;
        strcat(str, "Max speed set to %f.");
        ser->send_msg(str, val_f);
        break;
    case 0x53:
        this->position_pid.Kp = val_f;
        strcat(str, "Position P gain set to %f.");
        ser->send_msg(str, val_f);
        break;
    case 0x54:
        this->position_pid.Ki = val_f;
        strcat(str, "Position I gain set to %f.");
        ser->send_msg(str, val_f);
        break;
    case 0x55:
        this->position_pid.Kd = val_f;
        strcat(str, "Position D gain set to %f.");
        ser->send_msg(str, val_f);
        break;
    case 0x61:
        this->stiffness = val_f;
        strcat(str, "Stiffness set to %f.");
        ser->send_msg(str, val_f);
        break;
    case 0x62:
        this->damping = val_f;
        strcat(str, "Damping set to %f.");
        ser->send_msg(str, val_f);
        break;
    case 0x63:
        this->inertia = val_f;
        strcat(str, "Inertia set to %f.");
        ser->send_msg(str, val_f);
        break;
    case 0x71:
        this->timed_pulse_start_time = val_f;
        strcat(str, "Timed pulse start time set to %f.");
        ser->send_msg(str, val_f);
        break;
    case 0x72:
        this->timed_pulse_end_time = val_f;
        strcat(str, "Timed pulse stop time set to %f.");
        ser->send_msg(str, val_f);
        break;
    case 0x73:
        start_timer();
        strcat(str, "Timer started.");
        ser->send_msg(str);
        break;
    default:
        strcat(str, "Unknown command.");
        ser->send_msg(str);
        break;
    }
}