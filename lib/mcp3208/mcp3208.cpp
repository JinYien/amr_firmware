#include "mcp3208.h"

MCP3208::MCP3208(const uint16_t vref, const uint8_t csPin) : mVref(vref), mCsPin(csPin) {}

void MCP3208::setup() const
{
    pinMode(mCsPin, OUTPUT);
    digitalWriteFast(mCsPin, HIGH);
}

SpiData MCP3208::createCommand(const Channel ch)
{
    // base command structure
    // 0b000001cccc000000
    // c: channel config
    return SpiData{// add channel to basic command structure
                   .value = static_cast<uint16_t>((0x0400 | (ch << 6)))};
}

uint16_t MCP3208::read(const Channel ch)
{
    SpiData adc = {.value = 0};
    const SpiData cmd = createCommand(ch);

    // activate ADC with chip select
    digitalWriteFast(mCsPin, LOW);

    // send first command byte
    SPI.transfer(cmd.hiByte);
    // send second command byte and receive first(msb) 4 bits
    adc.hiByte = SPI.transfer(cmd.loByte) & 0x0F;
    // receive last(lsb) 8 bits
    adc.loByte = SPI.transfer(0x00);

    // deactivate ADC with slave select
    digitalWriteFast(mCsPin, HIGH);

    const uint16_t val_mV = toAnalog(adc.value);

    return val_mV;
}

uint16_t MCP3208::getVref() const { return mVref; }

uint8_t MCP3208::getCsPin() const { return mCsPin; }

uint16_t MCP3208::toAnalog(const uint16_t raw) const { return (static_cast<uint32_t>(raw) * mVref) / (kRes - 1); }

uint16_t MCP3208::toDigital(const uint16_t val) const { return (static_cast<uint32_t>(val) * (kRes - 1)) / mVref; }

uint16_t MCP3208::getAnalogRes() const { return (static_cast<uint32_t>(mVref) * 1000) / (kRes - 1); }
