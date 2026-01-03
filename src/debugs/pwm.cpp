#include <Arduino.h>
#include "defs.h"

#include "components/outputManager.h"

outputManager myPwm(MOTOR_RIGHT_DUTY_PIN, MOTOR_RIGHT_DIRECTION_PIN, MOTOR_LEFT_DUTY_PIN, MOTOR_LEFT_DIRECTION_PIN);

void readSerial();
void parseSerial(const String &str);

String serialString = ""; // string to hold incoming data
bool serialComplete = false;
bool streamData = false;

void setup()
{
    myPwm.setup();

    serialString.reserve(200);
    Serial.begin(115200);
    while (!Serial)
        ;
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
}

void parseSerial(const String &str)
{
    const char identifier = str.charAt(0);
    int val_t;
    double val_f;
    switch (identifier)
    {
    case 'd':
        val_t = str.substring(1).toInt();
        myPwm.set_duty(val_t, val_t);
        Serial.print("duty set to ");
        Serial.println(val_t);
        break;
    case 'c':
        val_f = str.substring(1).toFloat();
        myPwm.set_current(val_f, val_f);
        Serial.print("current set to ");
        Serial.println(val_f);
        Serial.print("duty set to ");
        Serial.println(myPwm.right_duty);
        break;
    }
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
