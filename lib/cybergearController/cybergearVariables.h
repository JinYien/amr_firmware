#ifndef CYBERGEAR_VARIABLES_H
#define CYBERGEAR_VARIABLES_H

#include <Arduino.h>
#include "cybergearCommands.h"

struct cybergearCommand
{
    uint8_t can_id;
    uint8_t cmd_id;
    uint16_t option;
    uint8_t len;
    uint8_t data[8];
};

struct cybergearMessage
{
    long rx_id;
    bool is_extended;
    bool is_rtr;
    uint8_t dlc;
    uint8_t data[8];
};

struct cybergearMotorParameter
{
    CONTROL_MODES run_mode;
    float iq_ref;
    float speed_ref;
    float limit_torque;
    float current_kp;
    float current_ki;
    float current_filter_gain;
    float loc_ref;
    float limit_speed;
    float limit_current;
    float mech_pos;
    float iqf;
    float mech_vel;
    float vbus;
    int16_t rotation;
    float loc_kp;
    float spd_kp;
    float spd_ki;
};

struct impedanceControlParameters
{
    float stiffness;
    float damping;
    float inertia;
};

class cybergearVariables
{
public:
    explicit cybergearVariables(uint8_t master_id = 0x00, uint8_t motor_id = 0x7F);

    uint8_t master_id = 0;
    uint8_t motor_id = 0;
    bool enabled = false;
    cybergearMotorParameter cybergear_motor_parameters{};
    impedanceControlParameters impedance_control_parameters{.stiffness = 1, .damping = 0, .inertia = 0};
    float friction_compensation = 0;

    uint8_t fault_data[8] = {};

    volatile float position{};    //!< encoder position (-4pi to 4pi)
    volatile float velocity{};    //!< motor velocity (-30rad/s to 30rad/s)
    volatile float torque{};      //!< motor torque (-12Nm - 12Nm)
    volatile float temperature{}; //!< temperature (degree C)

    volatile uint16_t raw_position{};    //!< raw position (for sync data)
    volatile uint16_t raw_velocity{};    //!< raw velocity (for sync data)
    volatile uint16_t raw_torque{};      //!< raw torque (for sync data)
    volatile uint16_t raw_temperature{}; //!< raw temperature (for sync data)

    cybergearCommand reset_motor() const;
    cybergearCommand enable_motor() const;
    cybergearCommand set_mech_position_to_zero() const;
    cybergearCommand set_run_mode(CONTROL_MODES run_mode) const;
    cybergearCommand change_motor_can_id(uint8_t new_can_id) const;

    void parse_packet(const cybergearMessage *msg);
    cybergearCommand read_control_params(CONTROL_PARAMS param) const;
    cybergearCommand write_control_params(CONTROL_PARAMS param, float value) const;

private:
    void process_response(const uint8_t *data, uint8_t len);
    void process_ram_read(const uint8_t *data, uint8_t len);
    void process_motor_fault(const uint8_t *data, uint8_t len);

    static int float_to_uint(float x, float x_min, float x_max, int bits);
    static float uint_to_float(uint16_t x, conversionRange range);
};

#endif // CYBERGEAR_VARIABLES_H
