#include "encoderManager.h"

encoderManager::encoderManager(uint8_t motor_right_qep, uint8_t motor_right_ch_a_pin, uint8_t motor_right_ch_b_pin,
                               uint8_t motor_left_qep, uint8_t motor_left_ch_a_pin, uint8_t motor_left_ch_b_pin) : qepRight(motor_right_qep, motor_right_ch_a_pin, motor_right_ch_b_pin, 0),
                                                                                                                   qepLeft(motor_left_qep, motor_left_ch_a_pin, motor_left_ch_b_pin, 0) {}

void encoderManager::setup(const double right_init_angle, const double left_init_angle,
                           const bool right_flexion_is_clockwise, const bool left_flexion_is_clockwise)
{

    this->my_right_flexion_is_clockwise = right_flexion_is_clockwise;
    this->my_left_flexion_is_clockwise = left_flexion_is_clockwise;

    this->qepRight.setInitConfig();
    this->qepRight.EncConfig.positionInitialValue = degrees_to_encoder_counts(right_init_angle);
    this->qepRight.EncConfig.enableReverseDirection = this->my_right_flexion_is_clockwise;
    this->qepRight.init();

    this->qepLeft.setInitConfig();
    this->qepLeft.EncConfig.positionInitialValue = degrees_to_encoder_counts(left_init_angle);
    this->qepLeft.EncConfig.enableReverseDirection = this->my_left_flexion_is_clockwise;
    this->qepLeft.init();
}

void encoderManager::set_initial_angle(const Side side, const double angle)
{
    const int32_t init_counts = degrees_to_encoder_counts(angle);
    if (side == Side::RIGHT)
    {
        this->qepRight.setInitConfig(); //
        this->qepRight.EncConfig.positionInitialValue = init_counts;
        this->qepRight.EncConfig.enableReverseDirection = this->my_right_flexion_is_clockwise;
        this->qepRight.init();
    }
    else if (side == Side::LEFT)
    {
        this->qepLeft.setInitConfig();
        this->qepLeft.EncConfig.positionInitialValue = init_counts;
        this->qepLeft.EncConfig.enableReverseDirection = this->my_left_flexion_is_clockwise;
        this->qepLeft.init();
    }
}

void encoderManager::update()
{
    this->right_value = this->qepRight.read();
    this->left_value = this->qepLeft.read();
    this->right_angle = encoder_counts_to_degrees(this->right_value);
    this->left_angle = encoder_counts_to_degrees(this->left_value);
}

double encoderManager::get_right_angle() const { return this->right_angle; }
double encoderManager::get_left_angle() const { return this->left_angle; }

double encoderManager::encoder_counts_to_degrees(const int32_t encoder_counts)
{
    return static_cast<double>(encoder_counts) / COUNTS_PER_GEAR_DEG;
}
int32_t encoderManager::degrees_to_encoder_counts(const double angle_deg)
{
    return static_cast<int32_t>(angle_deg * COUNTS_PER_GEAR_DEG);
}