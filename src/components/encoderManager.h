#ifndef ENCODER_MANAGER_H
#define ENCODER_MANAGER_H

#include "QuadEncoder.h"
#include "defs.h"

class encoderManager
{
public:
    encoderManager(uint8_t motor_right_qep, uint8_t motor_right_ch_a_pin, uint8_t motor_right_ch_b_pin,
                   uint8_t motor_left_qep, uint8_t motor_left_ch_a_pin, uint8_t motor_left_ch_b_pin);
    void setup(double right_init_angle = 0, double left_init_angle = 0, bool right_flexion_is_clockwise = false,
               bool left_flexion_is_clockwise = false);
    void set_initial_angle(Side side, double angle);
    double get_initial_angle(Side side);
    void update();
    double get_right_angle() const;
    double get_left_angle() const;

private:
    static double encoder_counts_to_degrees(int32_t encoder_counts);
    static int32_t degrees_to_encoder_counts(double angle_deg);
    QuadEncoder qepRight;
    QuadEncoder qepLeft;

    bool my_right_flexion_is_clockwise = false;
    bool my_left_flexion_is_clockwise = false;

    int32_t right_value = 0;
    int32_t left_value = 0;

    double right_angle = 0;
    double left_angle = 0;
};

#endif // ENCODER_MANAGER_H
