#include "xcorr.h"

namespace Seiscomp {
namespace detect {

bool xcorr(const double *tr1, const int size_tr1, const double *tr2,
           const int size_tr2, const double sampling_freq,
           const double max_lag_samples, Template::MatchResultPtr result) {

  auto SampleAtTrace = [=](int idx_template, int lag) {
    int idx_trace = idx_template + (size_tr2 - size_tr1) / 2 + lag;
    return (idx_trace < 0 || idx_trace >= size_tr2) ? 0 : tr2[idx_trace];
  };

  if (!result->squared_sum_template)
    return false;

  // do as much computation as possible outside the main cross-correlation loop
  double squared_sum_trace{0};
  for (int i = 0; i < size_tr1; ++i) {
    double sample{SampleAtTrace(i, -(max_lag_samples + 1))};
    squared_sum_trace += sample * sample;
  }

  // cross-correlation loop
  for (int lag = -max_lag_samples; lag <= max_lag_samples; ++lag) {
    // remove the sample that has exited the current xcorr win
    double last_sample_trace{SampleAtTrace(-1, lag)};
    squared_sum_trace -= last_sample_trace * last_sample_trace;
    // add the sample that has just entered the current xcorr win
    double new_sample_trace{SampleAtTrace(size_tr1 - 1, lag)};
    squared_sum_trace += new_sample_trace * new_sample_trace;

    double numerator{0};
    for (int i = 0; i < size_tr1; ++i) {
      numerator += tr1[i] * SampleAtTrace(i, lag);
    }

    const double denominator{
        std::sqrt(result->squared_sum_template * squared_sum_trace)};
    const double coeff{numerator / denominator};

    if (std::abs(coeff) > std::abs(result->coefficient) ||
        !std::isfinite(result->coefficient)) {
      result->numerator = numerator;
      result->squared_sum_trace = squared_sum_trace;
      result->coefficient = coeff;
      result->lag = lag / sampling_freq; // samples to secs
    }
  }

  if (!std::isfinite(result->coefficient)) {
    result->coefficient = 0;
    result->lag = 0;
    result->numerator = 0;
    result->squared_sum_template = 0;

    return false;
  }

  return true;
}

} // namespace detect
} // namespace Seiscomp
