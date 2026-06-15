#include "cybergearController.h"

cybergearController::cybergearController(const uint8_t num_of_motors, const uint8_t master_id, uint8_t *motor_ids,
                                         FlexCAN_T4_Base *my_can)
    : num_of_motors(num_of_motors), master_id(master_id), motor_ids(motor_ids), my_can(my_can)
{
    this->motors = new cybergearVariables[num_of_motors];
    for (uint8_t i = 0; i < this->num_of_motors; i++)
    {
        this->motors[i].master_id = master_id;
        this->motors[i].motor_id = motor_ids[i];
    }
};

void cybergearController::setup_motors()
{
    for (uint8_t i = 0; i < num_of_motors; i++)
    {
        reset_motor(i, true);
        set_mech_position_to_zero(i, true);
        enable_motor(i, true);

        read_control_params(i, CONTROL_PARAMS::RUN_MODE, true);
        read_control_params(i, CONTROL_PARAMS::IQ_REF, true);
        read_control_params(i, CONTROL_PARAMS::SPEED_REF, true);
        read_control_params(i, CONTROL_PARAMS::LIMIT_TORQUE, true);
        read_control_params(i, CONTROL_PARAMS::CURRENT_KP, true);
        read_control_params(i, CONTROL_PARAMS::CURRENT_KI, true);
        read_control_params(i, CONTROL_PARAMS::CURRENT_FILTER_GAIN, true);
        read_control_params(i, CONTROL_PARAMS::LOC_REF, true);
        read_control_params(i, CONTROL_PARAMS::LIMIT_SPEED, true);
        read_control_params(i, CONTROL_PARAMS::LIMIT_CURRENT, true);
        read_control_params(i, CONTROL_PARAMS::MECH_POS, true);
        read_control_params(i, CONTROL_PARAMS::IQF, true);
        read_control_params(i, CONTROL_PARAMS::VBUS, true);
        read_control_params(i, CONTROL_PARAMS::ROTATION, true);
        read_control_params(i, CONTROL_PARAMS::LOC_KP, true);
        read_control_params(i, CONTROL_PARAMS::SPD_KP, true);
        read_control_params(i, CONTROL_PARAMS::SPD_KI, true);
    }
}

void cybergearController::update_motor_statuses()
{
    this->replies_received = 0;
    for (uint8_t i = 0; i < num_of_motors; i++)
    {
        enable_motor(i, false);
    }
}

void cybergearController::reset_motor(const uint8_t motor_no, const bool wait_reply)
{
    const cybergearCommand new_cmd = this->motors[motor_no].reset_motor();
    send_command(&new_cmd);
    if (wait_reply)
    {
        wait_for_reply();
    }
}
void cybergearController::enable_motor(const uint8_t motor_no, const bool wait_reply)
{
    const cybergearCommand new_cmd = this->motors[motor_no].enable_motor();
    send_command(&new_cmd);
    if (wait_reply)
    {
        wait_for_reply();
    }
}

void cybergearController::set_mech_position_to_zero(const uint8_t motor_no, const bool wait_reply)
{
    const cybergearCommand new_cmd = this->motors[motor_no].set_mech_position_to_zero();
    send_command(&new_cmd);
    if (wait_reply)
    {
        wait_for_reply();
    }
}

void cybergearController::set_run_mode(const uint8_t motor_no, CONTROL_MODES run_mode, const bool wait_reply)
{
    const cybergearCommand new_cmd = this->motors[motor_no].set_run_mode(run_mode);
    send_command(&new_cmd);
    if (wait_reply)
    {
        wait_for_reply();
    }
}

void cybergearController::change_motor_can_id(const uint8_t motor_no, uint8_t new_can_id, const bool wait_reply)
{
    const cybergearCommand new_cmd = this->motors[motor_no].change_motor_can_id(new_can_id);
    send_command(&new_cmd);
    if (wait_reply)
    {
        wait_for_reply();
    }
}

void cybergearController::read_control_params(const uint8_t motor_no, const CONTROL_PARAMS param,
                                              const bool wait_reply)
{
    const cybergearCommand new_cmd = this->motors[motor_no].read_control_params(param);
    send_command(&new_cmd);
    if (wait_reply)
    {
        wait_for_reply();
    }
}

void cybergearController::write_control_params(const uint8_t motor_no, const CONTROL_PARAMS param, const float value,
                                               const bool wait_reply)
{
    const cybergearCommand new_cmd = this->motors[motor_no].write_control_params(param, value);
    send_command(&new_cmd);
    if (wait_reply)
    {
        wait_for_reply();
    }
}

void cybergearController::on_receive(const CAN_message_t &msg)
{
    cybergearMessage cybergear_msg{};

    cybergear_msg.is_extended = msg.flags.extended;
    cybergear_msg.is_rtr = msg.flags.remote;
    cybergear_msg.rx_id = msg.id;
    cybergear_msg.dlc = msg.len;

    for (uint8_t idx = 0; idx < cybergear_msg.dlc; ++idx)
    {
        cybergear_msg.data[idx] = msg.buf[idx];
    }

    const uint8_t receive_can_id = cybergear_msg.rx_id & 0xff;
    const uint8_t motor_can_id = (cybergear_msg.rx_id & 0xff00) >> 8;

    if (receive_can_id != this->master_id)
    {
        return;
    }

    for (uint8_t motor_no = 0; motor_no < this->num_of_motors; motor_no++)
    {
        if (this->motor_ids[motor_no] == motor_can_id)
        {
            this->motors[motor_no].parse_packet(&cybergear_msg);
            this->replies_received += 1;
            return;
        }
    }
}

bool cybergearController::wait_for_reply(const unsigned int num_of_replies, const uint32_t timeout_us)
{
    const uint32_t start = micros();
    while (this->replies_received < num_of_replies)
    {
        if (micros() - start > timeout_us)
        {
            this->replies_received = 0;
            return false;
        }
    }
    this->replies_received = 0;
    return true;
}
void cybergearController::send_command(const cybergearCommand *cmd) const
{
    const long id = cmd->cmd_id << 24 | cmd->option << 8 | cmd->can_id;

    CAN_message_t msg;
    msg.id = id;
    msg.len = cmd->len;
    msg.flags.extended = true;
    msg.flags.remote = false;

    for (int i = 0; i < msg.len; i++)
    {
        msg.buf[i] = cmd->data[i];
    }

    const uint32_t start = micros();
    while (this->my_can->write(msg) <= 0)
    {
        if (micros() - start > 1000)
        {
            break;
        }
    }
}

void cybergearController::update_motor_params(const uint8_t motor_no, const char id, const int val_t, const float val_f,
                                              serialManager *ser)
{
    char str[TX_BUFFER_SIZE];
    switch (id)
    {
    case 0x11:
        this->reset_motor(motor_no);
        snprintf(str, sizeof(str), "Cybergear %d: reset motor", motor_no);
        if (ser)
            ser->send_msg(str);
        break;
    case 0x12:
        this->enable_motor(motor_no);
        snprintf(str, sizeof(str), "Cybergear %d: enable motor", motor_no);
        if (ser)
            ser->send_msg(str);
        break;
    case 0x13:
        this->set_mech_position_to_zero(motor_no);
        snprintf(str, sizeof(str), "Cybergear %d: set zero position", motor_no);
        if (ser)
            ser->send_msg(str);
        break;
    case 0x14:
        if (val_t == 0)
        {
            this->set_run_mode(motor_no, CONTROL_MODES::CURRENT);
            snprintf(str, sizeof(str), "Cybergear %d: current control", motor_no);
            if (ser)
            {
                ser->send_msg(str);
            }
        }
        else if (val_t == 1)
        {
            this->set_run_mode(motor_no, CONTROL_MODES::SPEED);
            snprintf(str, sizeof(str), "Cybergear %d: speed control", motor_no);
            if (ser)
            {
                ser->send_msg(str);
            }
        }
        else if (val_t == 2)
        {
            this->set_run_mode(motor_no, CONTROL_MODES::POSITION);
            snprintf(str, sizeof(str), "Cybergear %d: position control", motor_no);
            if (ser)
            {
                ser->send_msg(str);
            }
        }
        break;
    case 0x21:
        this->write_control_params(motor_no, CONTROL_PARAMS::IQ_REF, val_f);
        snprintf(str, sizeof(str), "Cybergear %d: iq ref = %f", motor_no, val_f);
        if (ser)
            ser->send_msg(str);
        break;
    case 0x22:
        this->write_control_params(motor_no, CONTROL_PARAMS::SPEED_REF, val_f);
        snprintf(str, sizeof(str), "Cybergear %d: speed ref = %f", motor_no, val_f);
        if (ser)
            ser->send_msg(str);
        break;
    case 0x23:
        this->write_control_params(motor_no, CONTROL_PARAMS::LOC_REF, val_f);
        snprintf(str, sizeof(str), "Cybergear %d: loc ref = %f", motor_no, val_f);
        if (ser)
            ser->send_msg(str);
        break;
    case 0x31:
        this->write_control_params(motor_no, CONTROL_PARAMS::LIMIT_TORQUE, val_f);
        snprintf(str, sizeof(str), "Cybergear %d: limit torque = %f", motor_no, val_f);
        if (ser)
            ser->send_msg(str);
        break;
    case 0x32:
        this->write_control_params(motor_no, CONTROL_PARAMS::LIMIT_CURRENT, val_f);
        snprintf(str, sizeof(str), "Cybergear %d: limit current = %f", motor_no, val_f);
        if (ser)
            ser->send_msg(str);
        break;
    case 0x33:
        this->write_control_params(motor_no, CONTROL_PARAMS::LIMIT_SPEED, val_f);
        snprintf(str, sizeof(str), "Cybergear %d: limit speed = %f", motor_no, val_f);
        if (ser)
            ser->send_msg(str);
        break;
    case 0x41:
        this->write_control_params(motor_no, CONTROL_PARAMS::CURRENT_KP, val_f);
        snprintf(str, sizeof(str), "Cybergear %d: cur kp = %f", motor_no, val_f);
        if (ser)
            ser->send_msg(str);
        break;
    case 0x42:
        this->write_control_params(motor_no, CONTROL_PARAMS::CURRENT_KI, val_f);
        snprintf(str, sizeof(str), "Cybergear %d: cur ki = %f", motor_no, val_f);
        if (ser)
            ser->send_msg(str);
        break;
    case 0x43:
        this->write_control_params(motor_no, CONTROL_PARAMS::SPD_KP, val_f);
        snprintf(str, sizeof(str), "Cybergear %d: speed kp = %f", motor_no, val_f);
        if (ser)
            ser->send_msg(str);
        break;
    case 0x44:
        this->write_control_params(motor_no, CONTROL_PARAMS::SPD_KI, val_f);
        snprintf(str, sizeof(str), "Cybergear %d: speed ki = %f", motor_no, val_f);
        if (ser)
            ser->send_msg(str);
        break;
    case 0x45:
        this->write_control_params(motor_no, CONTROL_PARAMS::LOC_KP, val_f);
        snprintf(str, sizeof(str), "Cybergear %d: loc kp = %f", motor_no, val_f);
        if (ser)
            ser->send_msg(str);
        break;
    default:
        break;
    }
}

void cybergearController::reset_all_motors(const bool wait_reply)
{
    for (uint8_t motor_no = 0; motor_no < num_of_motors; motor_no++)
    {
        reset_motor(motor_no, wait_reply);
    }
}

void cybergearController::set_all_mech_position_to_zero(const bool wait_reply)
{
    for (uint8_t motor_no = 0; motor_no < num_of_motors; motor_no++)
    {
        set_mech_position_to_zero(motor_no, wait_reply);
    }
}
void cybergearController::set_all_run_mode(const CONTROL_MODES run_mode, const bool wait_reply)
{
    for (uint8_t motor_no = 0; motor_no < num_of_motors; motor_no++)
    {
        set_run_mode(motor_no, run_mode, wait_reply);
    }
}

void cybergearController::update_all_motor_params(const char id, const int val_t, const float val_f,
                                                  serialManager *ser)
{
    for (uint8_t motor_no = 0; motor_no < this->num_of_motors; motor_no++)
    {
        update_motor_params(motor_no, id, val_t, val_f, ser);
    }
}