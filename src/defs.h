#ifndef DEFS_H
#define DEFS_H

#include <Arduino.h>

// serial settings
constexpr int NUM_OF_VARIABLES = 13; // データの個数
constexpr int CHUNK_SIZE = 10;       // データグループの個数
constexpr int TX_BUFFER_SIZE = NUM_OF_VARIABLES * CHUNK_SIZE;
constexpr int RX_BUFFER_SIZE = 11;

// control settings
constexpr int SAMPLING_RATE_HZ = 1000;
constexpr double SAMPLING_PERIOD_SEC = 1.0 / static_cast<double>(SAMPLING_RATE_HZ);
constexpr int SAMPLING_PERIOD_USEC = 1'000'000 / SAMPLING_RATE_HZ;
constexpr double SPEED_FILTER_CUTOFF_FREQUENCY = 30; // Hz
constexpr double ACCELERATION_FILTER_CUTOFF_FREQUENCY = 30; // Hz

// adc, dac, spi settings
constexpr uint16_t VREF = 4096;           // 4.096V
constexpr uint32_t SPI_CLOCK = 1'200'000; // 1.2MHz

// adc settings
constexpr uint8_t ADC_CS_PIN = 14;
constexpr double ADC_12BIT_TO_VOLTAGE_GRADIENT = 0.004980915502865024;
constexpr double ADC_12BIT_TO_VOLTAGE_OFFSET = -10.249937733752171;

// qep settings
constexpr uint8_t MOTOR_RIGHT_QEP = 1;
constexpr uint8_t MOTOR_LEFT_QEP = 2;
constexpr uint8_t MOTOR_RIGHT_QEP_A_PIN = 1;
constexpr uint8_t MOTOR_RIGHT_QEP_B_PIN = 2;
constexpr uint8_t MOTOR_LEFT_QEP_A_PIN = 3;
constexpr uint8_t MOTOR_LEFT_QEP_B_PIN = 4;
constexpr bool MOTOR_RIGHT_FORWARD_CLOCKWISE = false;
constexpr bool MOTOR_LEFT_FORWARD_CLOCKWISE = true;
constexpr int PULSE_PER_TURN = 512;
constexpr int COUNTS_PER_MOTOR_TURN = PULSE_PER_TURN * 4;
constexpr double COUNTS_PER_MOTOR_DEG = static_cast<double>(COUNTS_PER_MOTOR_TURN) / 360;
constexpr double GEAR_RATIO = 36.0 / 1.0;
constexpr double COUNTS_PER_GEAR_DEG = COUNTS_PER_MOTOR_DEG * GEAR_RATIO;

// pwm pins
constexpr uint8_t MOTOR_RIGHT_DUTY_PIN = 5;
constexpr uint8_t MOTOR_RIGHT_DIRECTION_PIN = 7;
constexpr uint8_t MOTOR_LEFT_DUTY_PIN = 6;
constexpr uint8_t MOTOR_LEFT_DIRECTION_PIN = 8;

constexpr double MIN_CURRENT = 0;
constexpr double MAX_CURRENT = 4.13;
constexpr double TORQUE_CONSTANT_mNM_PER_A = 40.4;
constexpr double TORQUE_CONSTANT_NM_PER_A = TORQUE_CONSTANT_mNM_PER_A / 1E3;
constexpr int PWM_RESOLUTION = 12;         // bit
constexpr int PWM_RESOLUTION_VALUE = 4096; // 2^12
constexpr double MIN_DUTY = 0.1 * 4096;
constexpr double MAX_DUTY = 0.9 * 4096;
constexpr double DUTY_CURRENT_COEF = (MAX_DUTY - MIN_DUTY) / (MAX_CURRENT - MIN_CURRENT);

// cybergear settings
constexpr uint8_t NUM_MOTORS = 1;
constexpr uint8_t MASTER_CAN_ID = 0x00;
constexpr uint8_t MOTOR_CAN_ID = 0x7F;

// digital input pins
constexpr uint8_t STOP_BUTTON_PIN = 17;

// adc usage
constexpr uint8_t UNUSED_ADC_CH0 = 0;
constexpr uint8_t FORCE_X_ADC_CH = 1; // 力センサー（X軸）
constexpr uint8_t FORCE_Y_ADC_CH = 2; // 力センサー（Y軸）
constexpr uint8_t FORCE_Z_ADC_CH = 3; // 力センサー（Z軸）
constexpr uint8_t UNUSED_ADC_CH4 = 4;
constexpr uint8_t UNUSED_ADC_CH5 = 5;
constexpr uint8_t UNUSED_ADC_CH6 = 6;
constexpr uint8_t UNUSED_ADC_CH7 = 7;

enum class Side
{
    RIGHT = 0,
    LEFT = 1
};

#endif // DEFS_H
