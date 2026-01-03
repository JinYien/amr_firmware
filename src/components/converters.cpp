#include "converters.h"
#include <math.h>

linearlowpassConverter::linearlowpassConverter(const double gradient, const double offset) : gradient(gradient), offset(offset) {}

double linearlowpassConverter::convert(const double input)
{
    double signal;
    signal = input * gradient + offset;
    signal = this->signal_filter.filter(signal);
    return signal;
}
double linearlowpassConverter::linear_convert(const double input) const { return input * gradient + offset; }

void linearlowpassConverter::set_envelope_filter_cutoff_freq(double envelope_freq)
{
    if (envelope_freq < 0)
    {
        envelope_freq = 0;
    }
    else if (envelope_freq > sampling_freq / 2 - 1)
    {
        envelope_freq = sampling_freq / 2 - 1;
    }
    this->signal_filter.setup_filter(envelope_freq);
}
void linearlowpassConverter::set_gradient(const double gradient) { this->gradient = gradient; }
void linearlowpassConverter::set_offset(const double offset) { this->offset = offset; }
double linearlowpassConverter::get_gradient() const { return this->gradient; }
double linearlowpassConverter::get_offset() const { return this->offset; }
