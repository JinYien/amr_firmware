#pragma once

// https://thecodehound.com/butterworth-filter-design-in-c/
// https://www.dsprelated.com/showarticle/1119.php

#include <array>
#include <vector>

enum class FilterType
{
    LOWPASS = 0,
    HIGHPASS = 1,
    BANDPASS = 2,
};

template <unsigned int N>
class butterworthFilter
{
public:
    explicit butterworthFilter(FilterType filter_type = FilterType::LOWPASS, double fs = 1000, double fc1 = 30,
                               double fc2 = 0);
    void setup_filter(FilterType filter_type, double fs, double fc1, double fc2 = 0);
    void setup_filter(FilterType filter_type, double fc1, double fc2 = 0);
    void setup_filter(double fc1, double fc2 = 0);
    void setup_lowhighpass_filter(FilterType filter_type, double fs, double fc1);
    void setup_bandpass_filter(double fs, double fc1, double fc2);
    double filter(double x_new);
    std::vector<double> get_coeffs_a();
    std::vector<double> get_coeffs_b();

private:
    double fc1;
    double fc2;
    double fs;
    FilterType filter_type;
    std::array<double, N + 1> a_{};
    std::array<double, N + 1> b_{};
    std::array<double, N + 1> x_{}; // input history
    std::array<double, N + 1> y_{}; // output signal

    std::array<double, N * 2 + 1> a_band_{};
    std::array<double, N * 2 + 1> b_band_{};
    std::array<double, N * 2 + 1> x_band_{}; // input history
    std::array<double, N * 2 + 1> y_band_{}; // output signal
};
