#include <Arduino.h>

#include "TeensyTimerTool.h"
#include "components/serialManager.h"

using namespace TeensyTimerTool;
PeriodicTimer controlLoopTimer;

serialManager mySerial;
unsigned long start_time_usec = 0;
double sync_signal = 0;
double dummy_data = 0;

void control_loop();
void change_params(char id1, char id2, int64_t val_i, double val_f);

void setup()
{
    mySerial.set_callback(change_params);
    mySerial.start_serial();
    mySerial.enable_stream(false);

    start_time_usec = micros();
    controlLoopTimer.begin(control_loop, SAMPLING_PERIOD_USEC); // 1ms
}

void loop() { mySerial.read_serial(); }

void control_loop()
{
    const double run_time_sec = static_cast<double>(micros() - start_time_usec) / 1E6;
    const double data[NUM_OF_VARIABLES] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    mySerial.fill_chunk(data);
}

void change_params(const char id1, const char id2, const int64_t val_i, const double val_f)
{
    if (id1 == '@')
    {
        switch (id2)
        {
        case 0x09:
            mySerial.send_msg("Received command.");
            break;
        case 0x10:
            mySerial.send_msg("Received command ID %c.", id2);
            break;
        case 0x1E:
            mySerial.send_msg("Received int %d.", val_i);
            break;
        case 0x1F:
            mySerial.send_msg("Received double %f.", val_f);
            break;
        case 0x21:
            if (val_i == 1)
            {
                mySerial.send_msg("Starting data stream.");
                start_time_usec = micros();
                mySerial.enable_stream(true);
            }
            else if (val_i == 0)
            {
                mySerial.send_msg("Stopping data stream.");
                mySerial.enable_stream(false);
            }
            break;
        default:
            break;
        }
    }
}
