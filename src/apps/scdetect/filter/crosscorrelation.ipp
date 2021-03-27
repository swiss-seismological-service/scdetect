#ifndef SCDETECT_APPS_SCDETECT_FILTER_CROSSCORRELATION_IPP_
#define SCDETECT_APPS_SCDETECT_FILTER_CROSSCORRELATION_IPP_

#include <cfenv>
#include <cmath>
#include <string>

#include <boost/algorithm/string/join.hpp>

#include "../log.h"
#include "../utils.h"
#include "../waveform.h"

namespace Seiscomp {
namespace detect {
namespace filter {

template <typename TData>
CrossCorrelation<TData>::CrossCorrelation(const GenericRecordCPtr &template_wf,
                                          double sampling_freq)
    : sampling_frequency_{sampling_freq > 0 ? sampling_freq
                                            : template_wf->samplingFrequency()},
      template_wf_{template_wf} {

  if (sampling_frequency_ != template_wf->samplingFrequency()) {
    auto resampled{utils::make_smart<GenericRecord>(*template_wf_)};
    Resample(resampled, sampling_frequency_);
    template_wf_ = resampled;
  }

  Reset();
}

template <typename TData> CrossCorrelation<TData>::~CrossCorrelation() {}

template <typename TData>
void CrossCorrelation<TData>::Apply(size_t n_data, TData *data) {
  /*
   * Pearson correlation coefficient for time series X and Y of length n
   *
   *              sum((Xi-meanX) * (Yi-meanY))
   * cc = --------------------------------------------------
   *      sqrt(sum((Xi-meanX)^2)) * sqrt(sum((Yi-meanY)^2))
   *
   * Where sum(X)  is the sum of Xi for i=1 until i=n
   *
   * This can be rearranged in a form suitable for a single-pass algorithm
   * (where the mean of X and Y are not needed)
   *
   *                 n * sum(Xi*Yi) - sum(Xi) * sum(Yi)
   * cc = -----------------------------------------------------------
   *      sqrt(n*sum(Xi^2)-sum(Xi)^2) * sqrt(n*sum(Yi^2)-sum(Yi)^2))
   *
   * For cross-correlation, where we have a template waveform trace `tr1` which
   * is correlated against a data trace `tr2` at subsequent offset, we can
   * pre-compute the parts that involve `tr1` and re-use them at each step of
   * the cross-correlation:
   *
   *   sum_template_wf_ = sum(Xi)
   *   sum_squared_template_wf_= sum(Xi^2)
   *   denominator_template_wf_ =
   *     sqrt(n*sum_squared_template_wf_-(sum_template_wf_)^2)
   *
   * For the parts that involve the data trace (from the circular buffer) alone
   * we can compute them in a rolling fashion (removing first sample of
   * previous iteration and adding the last sample of the new iteration):
   *
   *   sum_data_ = sum(Yi)
   *   sum_squared_data_ = sum(Yi^2)
   *   denominator_data = sqrt(n*sum_squared_data_-(sum_data_)^2)
   *
   * Finally, this is the equation at each step (lag) of cross-correlation in
   * order to compute the Pearson correlation coefficient:
   *
   *       n * sum(Xi*Yi) - sum_template_wf_ * sum_data_
   * cc = -----------------------------------------------
   *        denominator_template_wf_ * denominator_data
   *
   * Unfortunately, we cannot optimize sum(Xi*Yi) and this will be a inner
   * loop inside the main cross-correlation loop
   */

  std::feclearexcept(FE_ALL_EXCEPT);

  const auto n{buffer_.capacity()};
  const TData *samples_template_wf{
      TypedArray<TData>::ConstCast(template_wf_->data())->typedData()};
  // cross correlation loop
  for (size_t i = 0; i < n_data; ++i) {
    const TData new_sample{data[i]};
    const TData last_sample{buffer_.front()};
    sum_data_ += new_sample - last_sample;
    sum_squared_data_ += new_sample * new_sample - last_sample * last_sample;
    const double denominator_data{
        std::sqrt(n * sum_squared_data_ - sum_data_ * sum_data_)};

    buffer_.push_back(new_sample);

    double sum_template_data{0};
    for (size_t k = 0; k < n; ++k) {
      sum_template_data += samples_template_wf[k] * buffer_[k];
    }

    const double pearson_coeff{
        (n * sum_template_data - sum_template_wf_ * sum_data_) /
        (denominator_template_wf_ * denominator_data)};

    int fe{fetestexcept(FE_ALL_EXCEPT)};
    if ((fe & ~FE_INEXACT) != 0) // we don't care about FE_INEXACT
    {
      std::vector<std::string> exceptions;
      if (fe & FE_DIVBYZERO)
        exceptions.push_back("FE_DIVBYZERO");
      if (fe & FE_INVALID)
        exceptions.push_back("FE_INVALID");
      if (fe & FE_OVERFLOW)
        exceptions.push_back("FE_OVERFLOW");
      if (fe & FE_UNDERFLOW)
        exceptions.push_back("FE_UNDERFLOW");

      std::string msg{
          "Floating point exception during cross-correlation (sample_idx=" +
          std::to_string(i) + ", sample=" + std::to_string(new_sample) + "): "};
      msg += boost::algorithm::join(exceptions, ", ");
      SCDETECT_LOG_WARNING(msg.c_str());

      std::feclearexcept(FE_ALL_EXCEPT);
    }

    data[i] =
        static_cast<TData>(std::isfinite(pearson_coeff) ? pearson_coeff : 0);
  }
}

template <typename TData>
void CrossCorrelation<TData>::Apply(std::vector<TData> &data) {
  Apply(data.size(), data.data());
}

template <typename TData>
void CrossCorrelation<TData>::Apply(TypedArray<TData> &data) {
  Apply(data.size(), data.typedData());
}

template <typename TData> void CrossCorrelation<TData>::Reset() {
  buffer_.clear();
  sum_squared_data_ = 0;
  sum_data_ = 0;

  const double *samples_template_wf{
      TypedArray<TData>::ConstCast(template_wf_->data())->typedData()};
  const int n{template_wf_->data()->size()};
  for (int i = 0; i < n; ++i) {
    sum_template_wf_ += samples_template_wf[i];
    sum_squared_template_wf_ += samples_template_wf[i] * samples_template_wf[i];
  }

  denominator_template_wf_ = std::sqrt(n * sum_squared_template_wf_ -
                                       sum_template_wf_ * sum_template_wf_);

  buffer_.set_capacity(n);
  while (!buffer_.full()) {
    buffer_.push_back(0);
  }
}

template <typename TData>
void CrossCorrelation<TData>::set_sampling_frequency(double sampling_freq) {
  if (sampling_freq != sampling_frequency_) {
    sampling_frequency_ = sampling_freq;

    auto resampled{utils::make_smart<GenericRecord>(*template_wf_)};
    Resample(resampled, sampling_frequency_);
    template_wf_ = resampled;

    Reset();
  }
}

template <typename TData>
double CrossCorrelation<TData>::sampling_frequency() const {
  return sampling_frequency_;
}

template <typename TData>
size_t CrossCorrelation<TData>::template_size() const {
  return template_wf_->sampleCount();
}

template <typename TData>
double CrossCorrelation<TData>::template_length() const {
  return template_wf_->timeWindow().length();
}

template <typename TData>
void CrossCorrelation<TData>::Resample(const GenericRecordPtr &wf,
                                       double sampling_freq) {
  waveform::Resample(*wf, sampling_freq, true);
}

} // namespace filter
} // namespace detect
} // namespace Seiscomp

#endif // SCDETECT_APPS_SCDETECT_FILTER_CROSSCORRELATION_IPP_
