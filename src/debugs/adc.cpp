#include <Arduino.h>
#include <SPI.h>
#include "components/adcManager.h"
#include "defs.h"

const SPISettings spiSettings(SPI_CLOCK, MSBFIRST, SPI_MODE0);
adcManager myAdc(VREF, ADC_CS_PIN);

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ;

    // setup SPI
    SPI.begin();
    myAdc.setup();

    SPI.beginTransaction(spiSettings);
}

void loop()
{
    // start time
    uint32_t t1 = micros();
    myAdc.update();
    // stop time
    uint32_t t2 = micros();

    Serial.printf("adc0Value: %d, adc1Value: %d, adc2Value: %d, adc3Value: %d\n(%d)\n",
                  myAdc.get_adc_value(0), myAdc.get_adc_value(1), myAdc.get_adc_value(2), myAdc.get_adc_value(3), t2 - t1);
    Serial.printf("adc0Voltage: %f, adc1Voltage: %f, adc2Voltage: %f, adc3Value%f\n(%d)\n",
                  myAdc.get_voltage(0), myAdc.get_voltage(1), myAdc.get_voltage(2), myAdc.get_adc_value(3), t2 - t1);

    delay(1000);
}
