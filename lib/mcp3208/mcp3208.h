/** @file mcp3208.h
 *
 * @brief  Interface for the Microchip MCP3208 12 bit ADC
 */

#pragma once

#include <Arduino.h>
#include <SPI.h>

typedef union spiData
{
    uint16_t value; /**< value */
    struct
    {
        uint8_t loByte; /**< low byte */
        uint8_t hiByte; /**< high byte */
    };
} SpiData;

constexpr size_t dmaSize = 2;
typedef struct dmaData
{
    volatile uint8_t src[dmaSize];
    volatile uint8_t dest[dmaSize];
} DmaData;

enum Channel
{
    SINGLE_0 = 0b1000, /**< single channel 0 */
    SINGLE_1 = 0b1001, /**< single channel 1 */
    SINGLE_2 = 0b1010, /**< single channel 2 */
    SINGLE_3 = 0b1011, /**< single channel 3 */
    SINGLE_4 = 0b1100, /**< single channel 4 */
    SINGLE_5 = 0b1101, /**< single channel 5 */
    SINGLE_6 = 0b1110, /**< single channel 6 */
    SINGLE_7 = 0b1111, /**< single channel 7 */
    DIFF_0PN = 0b0000, /**< differential channel 0 (input 0+,1-) */
    DIFF_0NP = 0b0001, /**< differential channel 0 (input 0-,1+) */
    DIFF_1PN = 0b0010, /**< differential channel 1 (input 2+,3-) */
    DIFF_1NP = 0b0011, /**< differential channel 1 (input 2-,3+) */
    DIFF_2PN = 0b0100, /**< differential channel 2 (input 4+,5-) */
    DIFF_2NP = 0b0101, /**< differential channel 2 (input 5-,5+) */
    DIFF_3PN = 0b0110, /**< differential channel 3 (input 6+,7-) */
    DIFF_3NP = 0b0111  /**< differential channel 3 (input 6-,7+) */
};

class MCP3208
{
public:
    /** ADC resolution in bits. */
    const uint8_t kResBits = 12;
    /** ADC resolution. */
    const uint16_t kRes = (1 << kResBits);

    MCP3208(uint16_t vref, uint8_t csPin);

    void setup() const;

    uint16_t read(Channel ch);
    uint16_t getVref() const;
    uint8_t getCsPin() const;
    uint16_t toDigital(uint16_t val) const;
    uint16_t getAnalogRes() const;
    uint16_t toAnalog(uint16_t raw) const;
    SpiData createCommand(Channel ch);

private:
    uint16_t mVref;
    uint8_t mCsPin;
};
