/** @file mcp4822.h
 *
 * @brief  Interface for the Microchip MCP4822 12 bit DAC
 */

#pragma once

#include <Arduino.h>

enum DACChannel : uint8_t
{
    Channel_A = 0u,
    Channel_B = 1u
};
enum DACGain : uint8_t
{
    High = 0u,
    Low = 1u
};

class MCP4822
{
public:
    /** ADC resolution in bits. */
    const uint8_t kResBits = 12;
    /** ADC resolution. */
    const uint16_t kRes = (1 << kResBits);

    MCP4822(uint16_t vref, uint8_t csPin1, uint8_t csPin2, uint8_t csPin3, uint8_t csPin4);

    void setup() const;
    void setValue(uint8_t dacNum, uint16_t value) const;

private:
    uint16_t createCommand(DACChannel channel, DACGain gain, uint16_t value) const;
    uint16_t mVref;
    uint8_t mCsPins[4];
};
