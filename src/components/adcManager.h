#ifndef ADC_MANAGER_H
#define ADC_MANAGER_H

#include "converters.h"
#include "mcp3208.h"
#include "serialManager.h"

enum class SignalType
{
    NONE = 0,
    LINEAR_LOWPASS = 1
};

class signalConverter
{
public:
    explicit signalConverter() = default;
    double convert(double input);
    void set_signal_type(SignalType signal_type);
    void set_signal_gradient(double gradient);
    void set_signal_offset(double offset);
    void set_envelope_filter_cutoff_freq(double envelope_freq);

private:
    SignalType signal_type = SignalType::LINEAR_LOWPASS;
    linearlowpassConverter linear_lowpass;
};

class adcManager
{
public:
    adcManager(uint16_t vref, uint8_t adc_cs_pin);
    void add_channel(Channel channel);
    void setup();
    void update();
    void set_signal_type(uint8_t channel_no, SignalType signal_type);
    void set_signal_gradient(uint8_t channel_no, double gradient);
    void set_signal_offset(uint8_t channel_no, double offset);

    void set_envelope_filter_cutoff_freq(uint8_t channel_no, double envelope_freq);

    [[nodiscard]] uint16_t get_adc_value(uint8_t channel_no) const;
    [[nodiscard]] double get_voltage(uint8_t channel_no) const;
    double get_converted_value(uint8_t channel_no);

    volatile bool update_completed = false;
    // AD変換器の電圧値
    double signal_values[8] = {};

    void change_params(char id, int64_t val_i, double val_f, serialManager *ser);

private:
    MCP3208 adc;
    linearlowpassConverter adc_values_to_voltage;
    uint8_t num_of_channels = 0;
    Channel channels[8] = {};
    uint16_t adc_values[8] = {};
    signalConverter converters[8];
};

#endif // ADC_MANAGER_H
