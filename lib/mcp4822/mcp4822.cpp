#include "mcp4822.h"
#include <SPI.h>

MCP4822::MCP4822(const uint16_t vref, const uint8_t csPin1, const uint8_t csPin2, const uint8_t csPin3,
                 const uint8_t csPin4) : mVref(vref) {
    mCsPins[0] = csPin1;
    mCsPins[1] = csPin2;
    mCsPins[2] = csPin3;
    mCsPins[3] = csPin4;
}

void MCP4822::setup() const {
    for (uint8_t i = 0; i < 4; i++) {
        const uint8_t pin = mCsPins[i];

        pinMode(pin, OUTPUT);
        digitalWriteFast(pin, HIGH);

        // set channel A to 0
        const uint16_t commandA = createCommand(DACChannel::Channel_A, DACGain::High, 0);
        digitalWrite(pin, LOW); // select device
        SPI.transfer16(commandA); // sent command for the A channel
        digitalWrite(pin, HIGH); // deselect device

        // set channel B to 0
        const uint16_t commandB = createCommand(DACChannel::Channel_B, DACGain::High, 0);
        digitalWrite(pin, LOW); // select device
        SPI.transfer16(commandB); // sent command for the A channel
        digitalWrite(pin, HIGH); // deselect device
    }
}

uint16_t MCP4822::createCommand(const DACChannel channel, const DACGain gain, uint16_t value) const {
    uint16_t command = 0;
    command = command | (channel << 15u) | (1u << 12u) | (gain << 13u);

    if (value > (1u << kResBits) - 1) {
        value = (1u << kResBits) - 1;
    } else {
        value = value << (12u - kResBits);
    }
    command = command & 0xF000u;
    command = command | value;
    return command;
}

void MCP4822::setValue(const uint8_t dacNum, const uint16_t value) const {
    const uint8_t pin = mCsPins[dacNum - 1];
    const uint16_t commandA = createCommand(DACChannel::Channel_A, DACGain::High, value);
    digitalWrite(pin, LOW); // select device
    SPI.transfer16(commandA); // sent command for the A channel
    digitalWrite(pin, HIGH); // deselect device
}
