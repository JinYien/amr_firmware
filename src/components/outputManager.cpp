#include "outputManager.h"
#include "defs.h"
#include <cmath>

outputManager::outputManager(const uint8_t right_duty_pin, const uint8_t right_direction_pin,
                             const uint8_t left_duty_pin, const uint8_t left_direction_pin) : right_duty_pin(right_duty_pin), right_direction_pin(right_direction_pin), left_duty_pin(left_duty_pin),
                                                                                              left_direction_pin(left_direction_pin) {}

void outputManager::setup() const
{
    pinMode(this->right_direction_pin, OUTPUT);
    pinMode(this->left_direction_pin, OUTPUT);

    analogWriteResolution(12);
    analogWrite(this->right_duty_pin, 0);
    analogWrite(this->left_duty_pin, 0);
}

void outputManager::set_duty(const int right_duty, const int left_duty)
{
    this->right_duty = right_duty;
    this->left_duty = left_duty;
    analogWrite(this->right_duty_pin, this->right_duty);
    analogWrite(this->left_duty_pin, this->left_duty);
}

void outputManager::set_direction(const Direction right_dir, const Direction left_dir) const
{
    digitalWriteFast(this->right_direction_pin, static_cast<uint8_t>(right_dir));
    digitalWriteFast(this->left_direction_pin, static_cast<uint8_t>(left_dir));
}

void outputManager::set_current(const double right_current, const double left_current)
{
    Direction right_dir = Direction::CLOCKWISE;
    Direction left_dir = Direction::CLOCKWISE;
    if (MOTOR_RIGHT_FORWARD_CLOCKWISE)
    {
        if (right_current < 0)
        {
            right_dir = Direction::CLOCKWISE;
        }
        else
        {
            right_dir = Direction::ANTICLOCKWISE;
        }
    }
    else
    {
        if (right_current < 0)
        {
            right_dir = Direction::ANTICLOCKWISE;
        }
        else
        {
            right_dir = Direction::CLOCKWISE;
        }
    }

    if (MOTOR_LEFT_FORWARD_CLOCKWISE)
    {
        if (left_current < 0)
        {
            left_dir = Direction::CLOCKWISE;
        }
        else
        {
            left_dir = Direction::ANTICLOCKWISE;
        }
    }
    else
    {
        if (left_current < 0)
        {
            left_dir = Direction::ANTICLOCKWISE;
        }
        else
        {
            left_dir = Direction::CLOCKWISE;
        }
    }
    set_duty(current_to_duty(std::abs(right_current)), current_to_duty(std::abs(left_current)));
    set_direction(right_dir, left_dir);
}

void outputManager::set_torque(const double right_torque, const double left_torque)
{
    this->right_torque = right_torque;
    this->left_torque = left_torque;
    const double right_current = right_torque / (TORQUE_CONSTANT_NM_PER_A * GEAR_RATIO);
    const double left_current = left_torque / (TORQUE_CONSTANT_NM_PER_A * GEAR_RATIO);
    set_current(right_current, left_current);
}

int outputManager::current_to_duty(double current)
{
    if (current < MIN_CURRENT)
    {
        current = MIN_CURRENT;
    }
    else if (current > MAX_CURRENT)
    {
        current = MAX_CURRENT;
    }
    return static_cast<int>(round(current * DUTY_CURRENT_COEF + MIN_DUTY));
}
