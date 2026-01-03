#include <Arduino.h>
#include "components/encoderManager.h"
#include "defs.h"

encoderManager myEncoder(MOTOR_RIGHT_QEP, MOTOR_RIGHT_QEP_A_PIN, MOTOR_RIGHT_QEP_B_PIN, MOTOR_LEFT_QEP,
                         MOTOR_LEFT_QEP_A_PIN, MOTOR_LEFT_QEP_B_PIN);

void setup()
{
    myEncoder.setup(0, 0, MOTOR_RIGHT_FORWARD_CLOCKWISE, MOTOR_LEFT_FORWARD_CLOCKWISE);

    Serial.begin(115200);
    while (!Serial)
        ;
    // myEncoder.set_initial_angle(Side::RIGHT, 20);
}

void loop()
{
    myEncoder.update();

    Serial.printf("encoderRight: %f, encoderLeft: %f\n", myEncoder.get_right_angle(), myEncoder.get_left_angle());
    delay(500);
}
