#include "adcManager.h"
#include <Arduino.h>
#include "defs.h"
#include "mcp3208.h"

double signalConverter::convert(const double input) { return this->linear_lowpass.convert(input); }

void signalConverter::set_signal_type(const SignalType signal_type) { this->signal_type = signal_type; }
void signalConverter::set_signal_gradient(const double gradient) { this->linear_lowpass.set_gradient(gradient); }
void signalConverter::set_signal_offset(const double offset) { this->linear_lowpass.set_offset(offset); }
void signalConverter::set_envelope_filter_cutoff_freq(const double envelope_freq)
{
    this->linear_lowpass.set_envelope_filter_cutoff_freq(envelope_freq);
}

adcManager::adcManager(const uint16_t vref, const uint8_t adc_cs_pin) : adc(vref, adc_cs_pin), adc_values_to_voltage(ADC_12BIT_TO_VOLTAGE_GRADIENT, ADC_12BIT_TO_VOLTAGE_OFFSET) {}

void adcManager::add_channel(const Channel channel) { this->channels[this->num_of_channels++] = channel; }

void adcManager::setup()
{
    this->adc.setup();
    add_channel(Channel::SINGLE_2);
    add_channel(Channel::SINGLE_3);
    add_channel(Channel::SINGLE_4);
    add_channel(Channel::SINGLE_5);
}

void adcManager::update()
{
    for (uint8_t i = 0; i < this->num_of_channels; i++)
    {
        this->adc_values[i] = adc.read(channels[i]);
        this->signal_values[i] = get_converted_value(i);
    }
    this->update_completed = true;
}

void adcManager::set_signal_type(const uint8_t channel_no, const SignalType signal_type)
{
    this->converters[channel_no].set_signal_type(signal_type);
}
void adcManager::set_signal_gradient(const uint8_t channel_no, const double gradient)
{
    this->converters[channel_no].set_signal_gradient(gradient);
}
void adcManager::set_signal_offset(const uint8_t channel_no, const double offset)
{
    this->converters[channel_no].set_signal_offset(offset);
}
void adcManager::set_envelope_filter_cutoff_freq(uint8_t channel_no, double envelope_freq)
{
    this->converters[channel_no].set_envelope_filter_cutoff_freq(envelope_freq);
}

uint16_t adcManager::get_adc_value(const uint8_t channel_no) const { return this->adc_values[channel_no]; }
double adcManager::get_voltage(const uint8_t channel_no) const
{
    return adc_values_to_voltage.linear_convert(this->get_adc_value(channel_no));
}
double adcManager::get_converted_value(const uint8_t channel_no)
{
    return this->converters[channel_no].convert(this->get_voltage(channel_no));
}

void adcManager::change_params(char const id, int64_t const val_i, const double val_f, serialManager *ser)
{
    switch (id)
    {
    case 0x11:
        set_signal_type(FORCE_X_ADC_CH, static_cast<SignalType>(val_i));
        ser->send_msg("Signal type of force sensor x-axis set to %d.", val_i);
        break;
    case 0x12:
        set_signal_type(FORCE_Y_ADC_CH, static_cast<SignalType>(val_i));
        ser->send_msg("Signal type of force sensor y-axis set to %d.", val_i);
        break;
    case 0x13:
        set_signal_type(FORCE_Z_ADC_CH, static_cast<SignalType>(val_i));
        ser->send_msg("Signal type of force sensor z-axis set to %d.", val_i);
        break;
    // Signal Gradient
    case 0x21:
        set_signal_gradient(FORCE_X_ADC_CH, val_f);
        ser->send_msg("Signal gradient of force sensor x-axis set to %f.", val_f);
        break;
    case 0x22:
        set_signal_gradient(FORCE_Y_ADC_CH, val_f);
        ser->send_msg("Signal gradient of force sensor y-axis set to %f.", val_f);
        break;
    case 0x23:
        set_signal_gradient(FORCE_Z_ADC_CH, val_f);
        ser->send_msg("Signal gradient of force sensor z-axis set to %f.", val_f);
        break;
    // Signal Offset
    case 0x31:
        set_signal_offset(FORCE_X_ADC_CH, val_f);
        ser->send_msg("Signal offset of force sensor x-axis set to %f.", val_f);
        break;
    case 0x32:
        set_signal_offset(FORCE_Y_ADC_CH, val_f);
        ser->send_msg("Signal offset of force sensor y-axis set to %f.", val_f);
        break;
    case 0x33:
        set_signal_offset(FORCE_Z_ADC_CH, val_f);
        ser->send_msg("Signal offset of force sensor z-axis set to %f.", val_f);
        break;
    default:
        ser->send_msg("Unknown parameter ID: 0x%02X", id);
    }
}