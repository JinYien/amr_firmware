#include <Arduino.h>
#include "components/cybergearController.h"
#include "defs.h"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

uint8_t motor_ids[NUM_MOTORS] = {MOTOR_CAN_ID};

cybergearController myCybergearController(NUM_MOTORS, MASTER_CAN_ID, motor_ids, &can1);

constexpr float CONTROL_FREQUENCY = 100;                                    // Hz
constexpr unsigned long CONTROL_INTERVAL = (1. / CONTROL_FREQUENCY) / 1E-6; // microseconds
unsigned long last_period = 0;

void readSerial();
void parseSerial(const String &str);
String serialString = ""; // string to hold incoming data
bool serialComplete = false;

bool streamData = false;
constexpr uint8_t SERIAL_PRESCALE = 2;
uint8_t serialPrescaleCounter = 0;

void change_params(char id1, char id2, int val_t, float val_f);
void on_receive(const CAN_message_t &msg) { myCybergearController.on_receive(msg); }
void periodic_loop();

void setup()
{
    serialString.reserve(200);
    Serial.begin(115200);
    while (!Serial)
        ;

    can1.begin();
    can1.setBaudRate(1000000);
    can1.setMaxMB(16);
    can1.enableFIFO();
    can1.enableFIFOInterrupt();
    Serial.println("CAN communication started...");

    // register the receive callback
    can1.onReceive(on_receive);

    myCybergearController.setup_motors();
    myCybergearController.set_run_mode(0, CONTROL_MODES::SPEED, true);
    myCybergearController.write_control_params(0, CONTROL_PARAMS::SPEED_REF, 2, true);
}

void loop()
{
    readSerial();
    if (serialComplete)
    {
        parseSerial(serialString);
        // clear the string
        serialString = "";
        serialComplete = false;
    }

    if (micros() - last_period >= CONTROL_INTERVAL)
    {
        last_period = micros();
        periodic_loop();
    }
}

void periodic_loop()
{
    myCybergearController.update_motor_statuses();
    const float current_pos = myCybergearController.motors[0].position;
    const float current_vel = myCybergearController.motors[0].velocity;

    if (streamData)
    {
        if (serialPrescaleCounter >= SERIAL_PRESCALE)
        {
            Serial.print("position is ");
            Serial.print(current_pos);
            Serial.print(", velocity is ");
            Serial.println(current_vel);
            serialPrescaleCounter = 0;
        }
    }
    serialPrescaleCounter++;
}

void parseSerial(const String &str)
{
    const char identifier = str.charAt(0);
    const int val_t = str.substring(1).toInt();
    const float val_f = str.substring(1).toFloat();
    if (identifier == 'a')
    {
        streamData = val_t;
    }
    myCybergearController.update_motor_params(0, identifier, val_t, val_f);
}

void readSerial()
{
    while (Serial.available())
    {
        if (const char inChar = static_cast<char>(Serial.read()); inChar == '\n')
            serialComplete = true;
        else
            serialString += inChar;
    }
}
