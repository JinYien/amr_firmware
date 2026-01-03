#ifndef OUTPUT_MANAGER_H
#define OUTPUT_MANAGER_H

#include <Arduino.h>

enum class Direction
{
    CLOCKWISE = LOW,
    ANTICLOCKWISE = HIGH,
};

class outputManager
{
public:
    outputManager(uint8_t right_duty_pin, uint8_t right_direction_pin, uint8_t left_duty_pin,
                  uint8_t left_direction_pin);
    void setup() const;
    void set_duty(int right_duty, int left_duty);
    void set_direction(Direction right_dir, Direction left_dir) const;
    void set_current(double right_current, double left_current);
    void set_torque(double right_torque, double left_torque);

    int right_duty = 0;
    int left_duty = 0;
    double right_torque = 0;
    double left_torque = 0;

private:
    static int current_to_duty(double current);
    uint8_t right_duty_pin;
    uint8_t right_direction_pin;
    uint8_t left_duty_pin;
    uint8_t left_direction_pin;
};

#endif // OUTPUT_MANAGER_H
