#ifndef CYBERGEAR_COMMANDS_H
#define CYBERGEAR_COMMANDS_H

#include <Arduino.h>

enum class COMMAND_TYPE
{
    POSITION = 1,
    RESPONSE = 2,
    ENABLE = 3,
    RESET = 4,
    SET_MECH_POSITION_TO_ZERO = 6,
    CHANGE_CAN_ID = 7,
    RAM_READ = 17,
    RAM_WRITE = 18,
    GET_MOTOR_FAIL = 21,
};

enum class CONTROL_PARAMS
{
    RUN_MODE = 0x7005,
    IQ_REF = 0x7006,
    SPEED_REF = 0x700A,
    LIMIT_TORQUE = 0x700B,
    CURRENT_KP = 0x7010,
    CURRENT_KI = 0x7011,
    CURRENT_FILTER_GAIN = 0x7014,
    LOC_REF = 0x7016,
    LIMIT_SPEED = 0x7017,
    LIMIT_CURRENT = 0x7018,
    MECH_POS = 0x7019,
    IQF = 0x701A,
    MECH_VEL = 0x701B,
    VBUS = 0x701C,
    ROTATION = 0x701D,
    LOC_KP = 0x701E, // not documented
    SPD_KP = 0x701F,
    SPD_KI = 0x7020,
};

enum class CONTROL_MODES
{
    MOTION = 0x00,
    POSITION = 0x01,
    SPEED = 0x02,
    CURRENT = 0x03,
};

struct conversionRange
{
    float min;
    float max;
};

namespace CONVERSION_RANGES
{
    constexpr conversionRange POSITION{-2 * PI, 2 * PI};
    constexpr conversionRange VELOCITY{-30, 30};
    constexpr conversionRange TORQUE{-12, 12};

    constexpr float KP_MIN = 0.0f;
    constexpr float KP_MAX = 500.0f;
    constexpr float KD_MIN = 0.0f;
    constexpr float KD_MAX = 5.0f;
    constexpr float T_MIN = -12.0f;
    constexpr float T_MAX = 12.0f;
    constexpr float IQ_MIN = -27.0f;
    constexpr float IQ_MAX = 27.0f;
    constexpr float CURRENT_FILTER_GAIN_MIN = 0.0f;
    constexpr float CURRENT_FILTER_GAIN_MAX = 1.0f;
}; // namespace CONVERSION_RANGES

namespace READ_WRITE_RANGES
{
    constexpr float IQ_REF_MAX = 23.0f;
    constexpr float IQ_REF_MIN = -23.0f;
    constexpr float SPD_REF_MAX = 30.0f;
    constexpr float SPD_REF_MIN = -30.0f;
    constexpr float LIMIT_TORQUE_MAX = 12.0f;
    constexpr float LIMIT_TORQUE_MIN = 0.0f;
    constexpr float CUR_KP_MAX = 200.0f;
    constexpr float CUR_KP_MIN = 0.0f;
    constexpr float CUR_KI_MAX = 200.0f;
    constexpr float CUR_KI_MIN = 0.0f;
    constexpr float LOC_KP_MAX = 200.0f;
    constexpr float LOC_KP_MIN = 0.0f;
    constexpr float SPD_KP_MAX = 200.0f;
    constexpr float SPD_KP_MIN = 0.0f;
    constexpr float LIMIT_SPD_MAX = 30.0f;
    constexpr float LIMIT_SPD_MIN = 0.0f;
    constexpr float LIMIT_CURRENT_MAX = 27.0f;
    constexpr float LIMIT_CURRENT_MIN = 0.0f;
}; // namespace READ_WRITE_RANGES

namespace DEFAULT_VALUES
{
    constexpr float CURRENT_KP = 0.125f;
    constexpr float CURRENT_KI = 0.0158f;
    constexpr float CURRENT_FINTER_GAIN = 0.1f;
    constexpr float POSITION_KP = 30.0f;
    constexpr float VELOCITY_KP = 2.0f;
    constexpr float VELOCITY_KI = 0.002f;
    constexpr float VELOCITY_LIMIT = 2.0f;
    constexpr float CURRENT_LIMIT = 27.0f;
    constexpr float TORQUE_LIMIT = 12.0f;
}; // namespace DEFAULT_VALUES

#endif // CYBERGEAR_COMMANDS_H
