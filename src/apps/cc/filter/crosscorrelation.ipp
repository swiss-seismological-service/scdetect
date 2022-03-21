#ifndef SCDETECT_APPS_CC_FILTER_CROSSCORRELATION_IPP_
#define SCDETECT_APPS_CC_FILTER_CROSSCORRELATION_IPP_

#include <seiscomp/core/strings.h>
#include <seiscomp/core/timewindow.h>

#include <boost/algorithm/string/join.hpp>
#include <cfenv>
#include <cmath>
#include <cstddef>
#include <memory>
#include <unordered_map>
#include <vector>

#include "../filter.h"
#include "../log.h"
#include "../util/math.h"
#include "crosscorrelation.h"

namespace Seiscomp {
namespace detect {
namespace filter {

template <typename TData>
CrossCorrelation<TData>::CrossCorrelation(const GenericRecordCPtr &waveform,
                                          std::shared_ptr<Executor> executor)
    : _templateWaveform{TemplateWaveform{waveform}},
      _executor{std::move(executor)} {
  setupFilter(waveform->samplingFrequency());
}

template <typename TData>
CrossCorrelation<TData>::CrossCorrelation(TemplateWaveform templateWaveform,
                                          std::shared_ptr<Executor> executor)
    : _templateWaveform{std::move(templateWaveform)},
      _executor{std::move(executor)} {
  if (_templateWaveform.processingConfig().samplingFrequency) {
    setupFilter(*_templateWaveform.processingConfig().samplingFrequency);
  }
}

template <typename TData>
void CrossCorrelation<TData>::apply(size_t nData, TData *data) {
  correlate(nData, data);
}

template <typename TData>
void CrossCorrelation<TData>::apply(std::vector<TData> &data) {
  apply(data.size(), data.data());
}

template <typename TData>
void CrossCorrelation<TData>::apply(TypedArray<TData> &data) {
  apply(data.size(), data.typedData());
}

template <typename TData>
void CrossCorrelation<TData>::reset() {
  _buffer.clear();

  _sumSquaredData = 0;
  _sumData = 0;

  const double *samples_template_wf{
      TypedArray<TData>::ConstCast(_templateWaveform.waveform().data())
          ->typedData()};
  const int n{_templateWaveform.waveform().data()->size()};
  _sumTemplateWaveform = 0;
  _sumSquaredTemplateWaveform = 0;
  for (int i = 0; i < n; ++i) {
    _sumTemplateWaveform += samples_template_wf[i];
    _sumSquaredTemplateWaveform += util::square(samples_template_wf[i]);
  }

  _denominatorTemplateWaveform = std::sqrt(n * _sumSquaredTemplateWaveform -
                                           util::square(_sumTemplateWaveform));

  _buffer = std::vector<TData>(n, 0);
}

template <typename TData>
void CrossCorrelation<TData>::setSamplingFrequency(double sampling_frequency) {
  setupFilter(sampling_frequency);
}

template <typename TData>
const TemplateWaveform &CrossCorrelation<TData>::templateWaveform() const {
  return _templateWaveform;
}

template <typename TData>
double CrossCorrelation<TData>::samplingFrequency() const {
  return _templateWaveform.samplingFrequency();
}

template <typename TData>
void CrossCorrelation<TData>::correlate(size_t nData, TData *data) {
  assert(_initialized);
  assert(_executor);

  /*
   * Pearson correlation coefficient for time series X and Y of length n
   *
   *              sum((Xi-meanX) * (Yi-meanY))
   * cc = --------------------------------------------------
   *      sqrt(sum((Xi-meanX)^2)) * sqrt(sum((Yi-meanY)^2))
   *
   * Where sum(X) is the sum of Xi for i=1 until i=n.
   *
   * This can be rearranged in a form suitable for a single-pass algorithm
   * (where the mean of X and Y are not needed):
   *
   *                 n * sum(Xi*Yi) - sum(Xi) * sum(Yi)
   * cc = -----------------------------------------------------------
   *      sqrt(n*sum(Xi^2)-sum(Xi)^2) * sqrt(n*sum(Yi^2)-sum(Yi)^2))
   *
   * Given a cross-correlation with a template waveform trace `tr1` which is
   * correlated against a data trace `tr2` at subsequent offset, pre-compute the
   * parts that involve `tr1` and re-use them at each step of the
   * cross-correlation:
   *
   *   _sumTemplateWaveform = sum(Xi)
   *   _sumSquaredTemplateWaveform= sum(Xi^2)
   *   _denominatorTemplateWaveform = \
   *     sqrt(n*_sumSquaredTemplateWaveform-(_sumTemplateWaveform)^2)
   *
   * For the parts that involve the data trace (extracted from the buffer)
   * exclusively, compute the components in a rolling fashion (removing first
   * sample of previous iteration and adding the last sample of the current
   * iteration):
   *
   *   _sumData = sum(Yi)
   *   _sumSquaredData = sum(Yi^2)
   *   denominator_data = sqrt(n*_sumSquaredData-(_sumData)^2)
   *
   * Finally, this is the equation at each step (lag) of cross-correlation in
   * order to compute the Pearson correlation coefficient:
   *
   *       n * sum(Xi*Yi) - _sumTemplateWaveform * _sumData
   * cc = --------------------------------------------------
   *       _denominatorTemplateWaveform * denominator_data
   *
   * Unfortunately, further optimization of sum(Xi*Yi) is not possible which
   * requires to be computed within an inner loop inside the main
   * cross-correlation loop.
   *
   * Note that Pearson correlation coefficients are computed concurrently, i.e.
   * the cross-correlation loop is parallelized and executed using the
   * _executor.
   */

  std::vector<double> sumsData;
  std::vector<double> sumsSquaredData;
  sumsData.reserve(nData);
  sumsSquaredData.reserve(nData);
  for (std::size_t i{0}; i < nData; ++i) {
    const TData newSample{data[i]};
    const TData lastSample{_buffer[i]};
    _sumData += newSample - lastSample;
    _sumSquaredData += util::square(newSample) - util::square(lastSample);

    sumsData.push_back(_sumData);
    sumsSquaredData.push_back(_sumSquaredData);

    _buffer.push_back(newSample);
  }

  const auto n{templateWaveform().size()};
  const TData *samplesTemplateWaveform{
      TypedArray<TData>::ConstCast(_templateWaveform.waveform().data())
          ->typedData()};

  auto loop = [this, n, data, samplesTemplateWaveform, &sumsData,
               &sumsSquaredData](const std::size_t &a, const std::size_t &b) {
    // cross-correlation loop
    for (std::size_t i{a}; i < b; ++i) {
      const double denominatorData{
          std::sqrt(n * sumsSquaredData[i] - util::square(sumsData[i]))};

      double sumTemplateData{0};
      for (std::size_t k{0}; k < n; ++k) {
        sumTemplateData += samplesTemplateWaveform[k] * _buffer[k + i + 1];
      }

      const double pearsonCoeff{
          (n * sumTemplateData - _sumTemplateWaveform * sumsData[i]) /
          (_denominatorTemplateWaveform * denominatorData)};

      data[i] =
          static_cast<TData>(std::isfinite(pearsonCoeff) ? pearsonCoeff : 0);
    }
  };

  const auto end{_buffer.size() - n};
  _executor->parallelize_loop(0, end, loop);

  // rollover buffer
  Buffer next{std::begin(_buffer) + end, std::end(_buffer)};
  _buffer = next;
}

template <typename TData>
void CrossCorrelation<TData>::setupFilter(double samplingFrequency) {
  assert((samplingFrequency > 0));

  _initialized = false;
  _templateWaveform.setSamplingFrequency(samplingFrequency);
  reset();
  _initialized = true;
}

}  // namespace filter
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_CC_FILTER_CROSSCORRELATION_IPP_
