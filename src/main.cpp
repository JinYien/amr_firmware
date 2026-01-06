#include <Arduino.h>
#include <SPI.h>
#include "TeensyTimerTool.h"
#include "components/adcManager.h"
#include "components/cybergearController.h"
#include "components/encoderManager.h"
#include "components/motorControl.h"
#include "components/outputManager.h"
#include "components/serialManager.h"
#include "defs.h"

using namespace TeensyTimerTool;
PeriodicTimer controlLoopTimer;
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

const SPISettings spiSettings(SPI_CLOCK, MSBFIRST, SPI_MODE0);
adcManager myAdc(VREF, ADC_CS_PIN);
encoderManager myEncoder(MOTOR_RIGHT_QEP, MOTOR_RIGHT_QEP_A_PIN, MOTOR_RIGHT_QEP_B_PIN, MOTOR_LEFT_QEP, MOTOR_LEFT_QEP_A_PIN, MOTOR_LEFT_QEP_B_PIN);
outputManager myOutputs(MOTOR_RIGHT_DUTY_PIN, MOTOR_RIGHT_DIRECTION_PIN, MOTOR_LEFT_DUTY_PIN, MOTOR_LEFT_DIRECTION_PIN);
serialManager mySerial;

uint8_t motor_ids[NUM_MOTORS] = {MOTOR_CAN_ID};
cybergearController myCybergearController(NUM_MOTORS, MASTER_CAN_ID, motor_ids, &can1);
motorControl rightMotor(Side::RIGHT);
motorControl leftMotor(Side::LEFT);

unsigned long previous_cybergear_millis = 0;
constexpr unsigned long CYBERGEAR_REFRESH_DUR = 10;

void change_params(char id1, char id2, int64_t val_i, double val_f);
void on_receive(const CAN_message_t &msg) { myCybergearController.on_receive(msg); }
void periodic_loop();

void setup()
{
    mySerial.set_callback(change_params);
    mySerial.start_serial();
    mySerial.enable_stream(false);

    can1.begin();
    can1.setBaudRate(1000000);
    can1.setMaxMB(16);
    can1.enableFIFO();
    can1.enableFIFOInterrupt();
    can1.onReceive(on_receive);

    myCybergearController.setup_motors();
    myEncoder.setup(0, 0, MOTOR_RIGHT_FORWARD_CLOCKWISE, MOTOR_LEFT_FORWARD_CLOCKWISE);
    myOutputs.setup();

    // Digital Input
    pinMode(17, INPUT_PULLDOWN);

    // SPI
    SPI.begin();
    myAdc.setup();

    // 角度センサー
    myAdc.set_signal_gradient(ANGLE_ADC_CH, 4.4326);
    myAdc.set_signal_offset(ANGLE_ADC_CH, 0);
    // 力センサー
    myAdc.set_signal_gradient(FORCE_X_ADC_CH, 3.9049);
    myAdc.set_signal_offset(FORCE_X_ADC_CH, -16.6);
    myAdc.set_signal_gradient(FORCE_Y_ADC_CH, 20.931);
    myAdc.set_signal_offset(FORCE_Y_ADC_CH, -52.365);
    myAdc.set_signal_gradient(FORCE_Z_ADC_CH, 19.6);
    myAdc.set_signal_offset(FORCE_Z_ADC_CH, -49.196);

    SPI.beginTransaction(spiSettings);

    controlLoopTimer.begin(periodic_loop, SAMPLING_PERIOD_USEC); // 1ms
}

void loop()
{
    mySerial.read_serial();
    unsigned long now = millis();
    if (now - previous_cybergear_millis >= CYBERGEAR_REFRESH_DUR)
    {
        myCybergearController.update_motor_statuses();
        previous_cybergear_millis = now;
    }
}
void periodic_loop()
{
    myEncoder.update();
    myAdc.update();

    const double stop_button = static_cast<double>(!digitalRead(17));
    if (stop_button > 0.5)
    {
        rightMotor.target_speed_deg_per_sec = 0.0;
        leftMotor.target_speed_deg_per_sec = 0.0;
        myCybergearController.write_control_params(0, CONTROL_PARAMS::SPEED_REF, 0.0f, false);
    }

    rightMotor.update(myEncoder.get_right_angle());
    leftMotor.update(myEncoder.get_left_angle());
    myOutputs.set_torque(rightMotor.get_torque_command_Nm(), leftMotor.get_torque_command_Nm());

    const double cybergear_position = myCybergearController.motors[0].position;
    const double cybergear_velocity = myCybergearController.motors[0].velocity;
    const double elevation_angle = myAdc.signal_values[ANGLE_ADC_CH];
    const double force_x = myAdc.signal_values[FORCE_X_ADC_CH];
    const double force_y = myAdc.signal_values[FORCE_Y_ADC_CH];
    const double force_z = myAdc.signal_values[FORCE_Z_ADC_CH];

    const double data[NUM_OF_VARIABLES] = {
        stop_button,
        rightMotor.get_angle_deg(),
        leftMotor.get_angle_deg(),
        rightMotor.get_speed_deg_per_sec(),
        leftMotor.get_speed_deg_per_sec(),
        myOutputs.right_torque,
        myOutputs.left_torque,
        cybergear_velocity,
        cybergear_position,
        elevation_angle,
        force_x,
        force_y,
        force_z,
    };
    mySerial.fill_chunk(data);
}

void change_params(const char id1, const char id2, const int64_t val_i, const double val_f)
{
    switch (id1)
    {
    case '@':
        switch (id2)
        {
        case 0x21:
            if (val_i == 1)
            {
                mySerial.send_msg("Starting data stream.");
                mySerial.enable_stream(true);
            }
            else if (val_i == 0)
            {
                mySerial.send_msg("Stopping data stream.");
                mySerial.enable_stream(false);
            }
            break;
        case 0x31:
            myEncoder.set_initial_angle(Side::RIGHT, val_f);
            mySerial.send_msg("Right angle (encoder) to %f.", val_f);
            break;
        case 0x32:
            myEncoder.set_initial_angle(Side::LEFT, val_f);
            mySerial.send_msg("Left angle (encoder) to %f.", val_f);
            break;
        case 0x41:
            rightMotor.start_timer();
            leftMotor.start_timer();
            mySerial.send_msg("Started all.");
            break;
        case 0x42:
            rightMotor.start_timer();
            leftMotor.start_timer();
            mySerial.send_msg("Started motors.");
            break;
        default:
            mySerial.send_msg("Unknown command.");
            break;
        }
        break;
    case '>':
        rightMotor.change_params(id2, val_i, val_f, &mySerial);
        break;
    case '<':
        leftMotor.change_params(id2, val_i, val_f, &mySerial);
        break;
    case '*':
        myCybergearController.update_motor_params(0, id2, val_i, val_f, &mySerial);
        break;
    case '%':
        myAdc.change_params(id2, val_i, val_f, &mySerial);
        break;
    default:
        mySerial.send_msg("Unknown command.");
    }
}
