#include "cybergearVariables.h"

cybergearVariables::cybergearVariables(const uint8_t master_id, const uint8_t motor_id) : master_id(master_id), motor_id(motor_id) {};

cybergearCommand cybergearVariables::reset_motor() const
{
    const cybergearCommand cmd = {.can_id = this->motor_id,
                                  .cmd_id = static_cast<uint8_t>(COMMAND_TYPE::RESET),
                                  .option = this->master_id,
                                  .len = 8,
                                  .data = {0x00}};
    return cmd;
}

cybergearCommand cybergearVariables::enable_motor() const
{
    const cybergearCommand cmd = {.can_id = this->motor_id,
                                  .cmd_id = static_cast<uint8_t>(COMMAND_TYPE::ENABLE),
                                  .option = this->master_id,
                                  .len = 8,
                                  .data = {0x00}};
    return cmd;
}
cybergearCommand cybergearVariables::set_mech_position_to_zero() const
{
    uint8_t data[8] = {0x00};
    data[0] = 0x01;
    cybergearCommand cmd = {.can_id = this->motor_id,
                            .cmd_id = static_cast<uint8_t>(COMMAND_TYPE::SET_MECH_POSITION_TO_ZERO),
                            .option = this->master_id,
                            .len = 8};
    memcpy(&cmd.data, &data, 8);
    return cmd;
}
cybergearCommand cybergearVariables::set_run_mode(const CONTROL_MODES run_mode) const
{
    uint8_t data[8] = {0x00};
    data[0] = static_cast<uint16_t>(CONTROL_PARAMS::RUN_MODE) & 0x00FF;
    data[1] = static_cast<uint16_t>(CONTROL_PARAMS::RUN_MODE) >> 8;
    data[4] = static_cast<uint8_t>(run_mode);
    cybergearCommand cmd = {.can_id = this->motor_id,
                            .cmd_id = static_cast<uint8_t>(COMMAND_TYPE::RAM_WRITE),
                            .option = this->master_id,
                            .len = 8};
    memcpy(&cmd.data, &data, 8);
    return cmd;
}

cybergearCommand cybergearVariables::change_motor_can_id(uint8_t new_can_id) const
{
    const uint16_t option = new_can_id << 8 | this->master_id;
    const cybergearCommand cmd = {.can_id = this->motor_id,
                                  .cmd_id = static_cast<uint8_t>(COMMAND_TYPE::CHANGE_CAN_ID),
                                  .option = option,
                                  .len = 8,
                                  .data = {0x00}};
    return cmd;
}

cybergearCommand cybergearVariables::read_control_params(const CONTROL_PARAMS param) const
{
    const auto addr = static_cast<uint16_t>(param);
    uint8_t data[8] = {0x00};
    memcpy(&data[0], &addr, 2);

    cybergearCommand cmd = {.can_id = this->motor_id,
                            .cmd_id = static_cast<uint8_t>(COMMAND_TYPE::RAM_READ),
                            .option = this->master_id,
                            .len = 8};
    memcpy(&cmd.data, &data, 8);

    return cmd;
}

cybergearCommand cybergearVariables::write_control_params(const CONTROL_PARAMS param, const float value) const
{
    const auto addr = static_cast<uint16_t>(param);
    uint8_t data[8] = {0x00};
    data[0] = addr & 0x00FF;
    data[1] = addr >> 8;
    memcpy(&data[4], &value, 4);

    cybergearCommand cmd = {.can_id = this->motor_id,
                            .cmd_id = static_cast<uint8_t>(COMMAND_TYPE::RAM_WRITE),
                            .option = this->master_id,
                            .len = 8};
    memcpy(&cmd.data, &data, 8);

    return cmd;
}

void cybergearVariables::parse_packet(const cybergearMessage *msg)
{
    const uint8_t receive_can_id = msg->rx_id & 0xff;
    const uint8_t motor_can_id = (msg->rx_id & 0xff00) >> 8;
    const uint8_t packet_type = (msg->rx_id & 0x3F000000) >> 24;

    if (motor_can_id != this->motor_id || receive_can_id != this->master_id)
    {
        // invalid id
        return;
    }

    if (static_cast<COMMAND_TYPE>(packet_type) == COMMAND_TYPE::RESPONSE)
    {
        process_response(msg->data, msg->dlc);
    }
    else if (static_cast<COMMAND_TYPE>(packet_type) == COMMAND_TYPE::RAM_READ)
    {
        process_ram_read(msg->data, msg->dlc);
    }
    else if (static_cast<COMMAND_TYPE>(packet_type) == COMMAND_TYPE::GET_MOTOR_FAIL)
    {
        process_motor_fault(msg->data, msg->dlc);
    }
    else
    {
        // unknown response
    }
}

void cybergearVariables::process_response(const uint8_t *data, uint8_t len)
{
    this->raw_position = data[1] | data[0] << 8;
    this->raw_velocity = data[3] | data[2] << 8;
    this->raw_torque = data[5] | data[4] << 8;
    this->raw_temperature = data[7] | data[6] << 8;

    this->position = uint_to_float(this->raw_position, CONVERSION_RANGES::POSITION);
    this->velocity = uint_to_float(this->raw_velocity, CONVERSION_RANGES::VELOCITY);
    this->torque = uint_to_float(this->raw_torque, CONVERSION_RANGES::TORQUE);
    this->temperature = static_cast<float>(this->raw_temperature) / 10;
}

void cybergearVariables::process_ram_read(const uint8_t *data, uint8_t len)
{
    uint16_t index = data[1] << 8 | data[0];

    uint8_t uint8_data;
    memcpy(&uint8_data, &data[4], sizeof(uint8_t));

    int16_t int16_data;
    memcpy(&int16_data, &data[4], sizeof(int16_t));

    float float_data;
    memcpy(&float_data, &data[4], sizeof(float));

    switch (static_cast<CONTROL_PARAMS>(index))
    {
    case CONTROL_PARAMS::RUN_MODE:
        this->cybergear_motor_parameters.run_mode = static_cast<CONTROL_MODES>(uint8_data);
        break;
    case CONTROL_PARAMS::IQ_REF:
        this->cybergear_motor_parameters.iq_ref = float_data;
        break;
    case CONTROL_PARAMS::SPEED_REF:
        this->cybergear_motor_parameters.speed_ref = float_data;
        break;
    case CONTROL_PARAMS::LIMIT_TORQUE:
        this->cybergear_motor_parameters.limit_torque = float_data;
        break;
    case CONTROL_PARAMS::CURRENT_KP:
        this->cybergear_motor_parameters.current_kp = float_data;
        break;
    case CONTROL_PARAMS::CURRENT_KI:
        this->cybergear_motor_parameters.current_ki = float_data;
        break;
    case CONTROL_PARAMS::CURRENT_FILTER_GAIN:
        this->cybergear_motor_parameters.current_filter_gain = float_data;
        break;
    case CONTROL_PARAMS::LOC_REF:
        this->cybergear_motor_parameters.loc_ref = float_data;
        break;
    case CONTROL_PARAMS::LIMIT_SPEED:
        this->cybergear_motor_parameters.limit_speed = float_data;
        break;
    case CONTROL_PARAMS::LIMIT_CURRENT:
        this->cybergear_motor_parameters.limit_current = float_data;
        break;
    case CONTROL_PARAMS::MECH_POS:
        this->cybergear_motor_parameters.mech_pos = float_data;
        break;
    case CONTROL_PARAMS::IQF:
        this->cybergear_motor_parameters.iqf = float_data;
        break;
    case CONTROL_PARAMS::MECH_VEL:
        this->cybergear_motor_parameters.mech_vel = float_data;
        break;
    case CONTROL_PARAMS::VBUS:
        this->cybergear_motor_parameters.vbus = float_data;
        break;
    case CONTROL_PARAMS::ROTATION:
        this->cybergear_motor_parameters.rotation = int16_data;
        break;
    case CONTROL_PARAMS::LOC_KP:
        this->cybergear_motor_parameters.loc_kp = float_data;
        break;
    case CONTROL_PARAMS::SPD_KP:
        this->cybergear_motor_parameters.spd_kp = float_data;
        break;
    case CONTROL_PARAMS::SPD_KI:
        this->cybergear_motor_parameters.spd_ki = float_data;
        break;
    default:
        break;
    }
}

void cybergearVariables::process_motor_fault(const uint8_t *data, const uint8_t len)
{
    memcpy(this->fault_data, data, sizeof(uint8_t) * len);
    // TODO
}

int cybergearVariables::float_to_uint(const float x, const float x_min, const float x_max, const int bits)
{
    const float span = x_max - x_min;
    const float offset = x_min;
    float sat_x = x;
    if (x > x_max)
    {
        sat_x = x_max;
    }
    else if (x < x_min)
    {
        sat_x = x_min;
    }
    return static_cast<int>((sat_x - offset) * static_cast<float>((1 << bits) - 1) / span);
}

float cybergearVariables::uint_to_float(const uint16_t x, conversionRange const range)
{
    constexpr uint16_t type_max = 0xFFFF;
    const float span = range.max - range.min;
    return static_cast<float>(x) / type_max * span + range.min;
}
