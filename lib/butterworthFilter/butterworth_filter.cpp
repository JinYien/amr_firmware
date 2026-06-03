#include "butterworth_filter.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <complex>
#include <numeric>
#include <vector>

constexpr double PI = 3.1415926535897932384626433832795;
constexpr double HALF_PI = 1.5707963267948966192313216916398;

using namespace std::literals::complex_literals;

using Polynomial = std::vector<std::complex<double>>;

Polynomial operator*(const Polynomial &p, const Polynomial &q)
{
    const size_t n = p.size() + q.size() - 1;
    Polynomial result(n);
    for (size_t i = 0; i < p.size(); i++)
        for (size_t j = 0; j < q.size(); j++)
            result[i + j] += p[i] * q[j];
    return result;
}

// Emulate Matlab 'poly' function.  Calculate the coefficients of the polynomial
// with the specified roots.
static std::vector<std::complex<double>> poly(const std::vector<std::complex<double>> &roots)
{
    Polynomial result{std::complex<double>{1.0, 0.0}};
    for (auto &r : roots)
    {
        Polynomial factor{
            std::complex<double>(-r.real(), -r.imag()), // –r
            std::complex<double>(1.0, 0.0)              //  x-term
        };
        result = result * factor;
    }

    // Matlab returns the highest order coefficients first.
    std::reverse(result.begin(), result.end());
    return result;
}

// Emulate Matlab 'sum' function.
static std::complex<double> sum(const std::vector<std::complex<double>> &v)
{
    return std::accumulate(v.begin(), v.end(), 0.0i);
}

// Convolve two complex sequences
std::vector<std::complex<double>> convolve(const std::vector<std::complex<double>> &x,
                                           const std::vector<std::complex<double>> &y)
{
    std::vector<std::complex<double>> result(x.size() + y.size() - 1, std::complex<double>(0.0, 0.0));
    for (size_t i = 0; i < x.size(); ++i)
    {
        for (size_t j = 0; j < y.size(); ++j)
        {
            result[i + j] += x[i] * y[j];
        }
    }
    return result;
}

template <unsigned int N>
butterworthFilter<N>::butterworthFilter(const FilterType filter_type, const double fs, const double fc1,
                                        const double fc2) : fc1(fc1), fc2(fc2), fs(fs), filter_type(filter_type)
{
    setup_filter(filter_type, fs, fc1, fc2);
}

template <unsigned int N>
void butterworthFilter<N>::setup_filter(const FilterType filter_type, const double fs, const double fc1,
                                        const double fc2)
{
    if (filter_type == FilterType::LOWPASS or filter_type == FilterType::HIGHPASS)
    {
        if (fc1 < fs / 2)
        { // Cutoff frequency must be lower that fs/2
            setup_lowhighpass_filter(filter_type, fs, fc1);
        }
    }
    else if (filter_type == FilterType::BANDPASS)
    {
        if (fc1 < fc2 and fc2 < fs / 2)
        { // Cutoff frequencies must be lower than fc2
            setup_bandpass_filter(fs, fc1, fc2);
        }
    }
}

template <unsigned int N>
void butterworthFilter<N>::setup_filter(const FilterType filter_type, const double fc1, const double fc2)
{
    setup_filter(filter_type, this->fs, fc1, fc2);
}

template <unsigned int N>
void butterworthFilter<N>::setup_filter(const double fc1, const double fc2)
{
    setup_filter(this->filter_type, this->fs, fc1, fc2);
}

template <unsigned int N>
void butterworthFilter<N>::setup_lowhighpass_filter(const FilterType filter_type, const double fs, const double fc1)
{
    this->filter_type = filter_type;
    this->fs = fs;
    this->fc1 = fc1;

    std::vector<std::complex<double>> pa(N);
    std::vector<std::complex<double>> p(N);
    std::vector<std::complex<double>> q(N, {-1.0, 0});
    assert(fc1 < fs / 2); // Cutoff frequency must be lower that fs/2
    assert(filter_type == FilterType::LOWPASS or filter_type == FilterType::HIGHPASS);

    // I. Find poles of analog filter
    for (unsigned int i = 0; i < N; i++)
    {
        const unsigned int k = i + 1;
        const double theta = (2 * k - 1) * HALF_PI / N;
        pa[i] = -sin(theta) + 1.0i * cos(theta);
    }

    // II. Scale poles in frequency
    const double Fc = fs / PI * tan(PI * fc1 / fs);
    if (filter_type == FilterType::LOWPASS)
    {
        for (auto &i : pa)
            i *= 2 * PI * Fc;
    }
    else if (filter_type == FilterType::HIGHPASS)
    {
        for (size_t i = 0; i < pa.size(); i++)
        {
            pa[i] = (2 * PI * Fc) / pa[i]; // correctly invert each pole
            q[i] = -q[i];                  // flip zeros from –1 to +1
        }
    }

    // III. Find coeffs of digital filter poles and zeros in the z plane
    for (size_t i = 0; i < N; i++)
        p[i] = (1.0 + pa[i] / (2 * fs)) / (1.0 - pa[i] / (2 * fs));

    auto a = poly(p);
    for (auto &i : a)
        i = i.real();

    auto b = poly(q);
    std::complex<double> K;
    if (filter_type == FilterType::LOWPASS)
    {
        K = sum(a) / sum(b);
    }
    else if (filter_type == FilterType::HIGHPASS)
    {
        std::complex<double> num = {0.0, 0.0}, den = {0.0, 0.0};
        const int M = static_cast<int>(a.size()) - 1;
        for (size_t i = 0; i < a.size(); ++i)
        {
            double w = ((M - static_cast<int>(i)) & 1) ? -1.0 : 1.0;
            num += a[i] * w;
            den += b[i] * w;
        }
        K = num / den;
    }
    for (auto &i : b)
        i *= K;

    for (int i = 0; i <= N; i++)
    {
        this->a_[i] = a[i].real();
        this->b_[i] = b[i].real();
    }

    std::fill(std::begin(this->x_), std::end(this->x_), 0);
    std::fill(std::begin(this->y_), std::end(this->y_), 0);
}

template <unsigned int N>
void butterworthFilter<N>::setup_bandpass_filter(const double fs, const double fc1, double fc2)
{
    this->fs = fs;
    this->fc1 = fc1;
    this->fc2 = fc2;
    this->filter_type = FilterType::BANDPASS;
    assert(fc1 < fc2);
    assert(fc2 < fs / 2);

    // Pre-warp frequencies
    const double Fc1 = 2.0 * fs * std::tan(PI * fc1 / fs);
    const double Fc2 = 2.0 * fs * std::tan(PI * fc2 / fs);
    const double B = Fc2 - Fc1;
    const double Fc0 = std::sqrt(Fc1 * Fc2);

    // Analog prototype poles
    std::vector<std::complex<double>> pa_prototype;
    for (unsigned int k = 1; k <= N; ++k)
    {
        std::complex<double> pole = std::polar(1.0, HALF_PI + (2.0 * k - 1.0) * HALF_PI / N);
        pa_prototype.push_back(pole);
    }

    // Bandpass transformation
    std::vector<std::complex<double>> pa_bandpass;
    for (const auto &sp : pa_prototype)
    {
        auto term = std::sqrt(std::pow(B * sp, 2) - 4.0 * Fc0 * Fc0);
        auto root1 = (B * sp + term) / 2.0;
        auto root2 = (B * sp - term) / 2.0;
        pa_bandpass.push_back(root1);
        pa_bandpass.push_back(root2);
    }

    // Bilinear transform
    std::vector<std::complex<double>> p_digital;
    for (const auto &s : pa_bandpass)
    {
        auto z = (1.0 + s / (2.0 * fs)) / (1.0 - s / (2.0 * fs));
        p_digital.push_back(z);
    }

    // Zeros at z=1 and z=-1
    std::vector<std::complex<double>> zeros;
    for (unsigned int i = 0; i < N; ++i)
    {
        zeros.push_back(std::complex<double>{1.0, 0.0});
        zeros.push_back(std::complex<double>{-1.0, 0.0});
    }

    // Compute polynomials
    const auto a_poly = poly(p_digital);
    auto b_poly = poly(zeros);

    // Gain adjustment
    const double f0 = std::sqrt(fc1 * fc2);
    const double omega0 = 2.0 * PI * f0 / fs;

    std::complex<double> sum_b = {0.0, 0.0}, sum_a = {0.0, 0.0};
    for (size_t i = 0; i < b_poly.size(); ++i)
    {
        sum_b += b_poly[i] * std::exp(std::complex<double>(0, -omega0 * i));
    }
    for (size_t i = 0; i < a_poly.size(); ++i)
    {
        sum_a += a_poly[i] * std::exp(std::complex<double>(0, -omega0 * i));
    }

    const std::complex<double> K = sum_a / sum_b;

    for (auto &coeff : b_poly)
    {
        coeff *= K;
    }

    // Assign to member arrays
    for (unsigned int i = 0; i <= 2 * N; ++i)
    {
        this->a_band_[i] = (i < a_poly.size()) ? a_poly[i].real() : 0.0;
        this->b_band_[i] = (i < b_poly.size()) ? b_poly[i].real() : 0.0;
    }

    std::fill(std::begin(this->x_band_), std::end(this->x_band_), 0);
    std::fill(std::begin(this->y_band_), std::end(this->y_band_), 0);
}

template <unsigned int N>
double butterworthFilter<N>::filter(const double x_new)
{
    if (this->filter_type == FilterType::BANDPASS)
    {
        // Shift input history
        for (int i = N * 2; i > 0; --i)
        {
            this->x_band_[i] = this->x_band_[i - 1];
        }
        this->x_band_[0] = x_new;

        // 2) compute forward and backward sums with correct indexing
        double y_new = 0.0;
        for (int i = 0; i <= N * 2; ++i)
        {
            y_new += this->b_band_[i] * this->x_band_[i];
        }
        for (int i = 1; i <= N * 2; ++i)
        {
            y_new -= this->a_band_[i] * this->y_band_[i - 1];
        }
        // 3) normalize if needed
        y_new /= this->a_band_[0];

        // Shift output history
        for (int j = N * 2 - 1; j > 0; --j)
        {
            this->y_band_[j] = this->y_band_[j - 1];
        }
        this->y_band_[0] = y_new;

        return y_new;
    }
    else
    {
        // Shift input history
        for (unsigned int i = N; i > 0; --i)
        {
            this->x_[i] = this->x_[i - 1];
        }
        this->x_[0] = x_new;

        // 2) compute forward and backward sums with correct indexing
        double y_new = 0.0;
        for (unsigned int i = 0; i <= N; ++i)
        {
            y_new += this->b_[i] * this->x_[i];
        }
        for (int i = 1; i <= N; ++i)
        {
            y_new -= this->a_[i] * this->y_[i - 1];
        }
        // 3) normalize if needed
        y_new /= this->a_[0];

        // Shift output history
        for (unsigned int j = N - 1; j > 0; --j)
        {
            this->y_[j] = this->y_[j - 1];
        }
        this->y_[0] = y_new;

        return y_new;
    }
}

template <unsigned int N>
std::vector<double> butterworthFilter<N>::get_coeffs_a()
{
    std::vector<double> out{};
    if (this->filter_type == FilterType::BANDPASS)
    {
        for (auto &i : this->a_band_)
        {
            out.push_back(i);
        }
    }
    else
    {
        for (auto &i : this->a_)
        {
            out.push_back(i);
        }
    }
    return out;
}

template <unsigned int N>
std::vector<double> butterworthFilter<N>::get_coeffs_b()
{
    std::vector<double> out{};
    if (this->filter_type == FilterType::BANDPASS)
    {
        for (auto &i : this->b_band_)
        {
            out.push_back(i);
        }
    }
    else
    {
        for (auto &i : this->b_)
        {
            out.push_back(i);
        }
    }
    return out;
}

template class butterworthFilter<2>;
template class butterworthFilter<4>;
template class butterworthFilter<8>;
