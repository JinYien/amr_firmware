#ifndef CONVERTERS_H
#define CONVERTERS_H

#include "butterworth_filter.h"

class linearlowpassConverter
{
public:
    explicit linearlowpassConverter(double gradient = 1.0, double offset = 0.0);
    double convert(double input);
    double linear_convert(double input) const;
    void set_gradient(double gradient);
    void set_offset(double offset);
    double get_gradient() const;
    double get_offset() const;
    void set_envelope_filter_cutoff_freq(double envelope_freq);

private:
    double gradient;
    double offset;
    double sampling_freq = 1000;
    double envelope_filter_freq = 8;
    butterworthFilter<2> signal_filter = butterworthFilter<2>(FilterType::LOWPASS, sampling_freq, envelope_filter_freq);
};

#endif // CONVERTERS_H