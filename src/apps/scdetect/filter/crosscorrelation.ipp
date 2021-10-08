#ifndef SCDETECT_APPS_SCDETECT_FILTER_CROSSCORRELATION_IPP_
#define SCDETECT_APPS_SCDETECT_FILTER_CROSSCORRELATION_IPP_

#include <seiscomp/core/strings.h>
#include <seiscomp/core/timewindow.h>

#include <boost/algorithm/string/join.hpp>
#include <cfenv>
#include <cmath>
#include <string>

#include "../filter.h"
#include "../log.h"
#include "../util/math.h"
#include "../util/memory.h"
#include "../waveform.h"

namespace Seiscomp {
namespace detect {
namespace filter {

template <typename TData>
CrossCorrelation<TData>::CrossCorrelation() {}

template <typename TData>
CrossCorrelation<TData>::CrossCorrelation(const GenericRecordCPtr &waveform)
    : _initialized{true},
      _templateWaveform{waveform},
      _samplingFrequency{waveform->samplingFrequency()} {
  setupFilter(*_samplingFrequency);
}

template <typename TData>
CrossCorrelation<TData>::~CrossCorrelation() {}

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
      TypedArray<TData>::ConstCast(_templateWaveform->data())->typedData()};
  const int n{_templateWaveform->data()->size()};
  _sumTemplateWaveform = 0;
  _sumSquaredTemplateWaveform = 0;
  for (int i = 0; i < n; ++i) {
    _sumTemplateWaveform += samples_template_wf[i];
    _sumSquaredTemplateWaveform += util::square(samples_template_wf[i]);
  }

  _denominatorTemplateWaveform =
      std::sqrt(n * _sumSquaredTemplateWaveform -
                _sumTemplateWaveform * _sumTemplateWaveform);

  _buffer.set_capacity(n);
  while (!_buffer.full()) {
    _buffer.push_back(0);
  }
}

template <typename TData>
void CrossCorrelation<TData>::setSamplingFrequency(double sampling_frequency) {
  setupFilter(sampling_frequency);
}

template <typename TData>
double CrossCorrelation<TData>::samplingFrequency() const {
  return _samplingFrequency.value_or(0);
}

template <typename TData>
size_t CrossCorrelation<TData>::templateSize() const {
  return _initialized ? _templateWaveform->sampleCount() : 0;
}

template <typename TData>
double CrossCorrelation<TData>::templateLength() const {
  return _initialized ? _templateWaveform->timeWindow().length() : 0;
}

template <typename TData>
boost::optional<const Core::Time> CrossCorrelation<TData>::templateStartTime()
    const {
  if (_initialized) {
    return _templateWaveform->startTime();
  }
  return boost::none;
}

template <typename TData>
boost::optional<const Core::Time> CrossCorrelation<TData>::templateEndTime()
    const {
  if (_initialized) {
    _templateWaveform->endTime();
  }
  return boost::none;
}

template <typename TData>
void CrossCorrelation<TData>::correlate(size_t nData, TData *data) {
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
   * For the parts that involve the data trace (extracted from the circular
   * buffer) exclusively, compute the components in a rolling fashion (removing
   * first sample of previous iteration and adding the last sample of the
   * current iteration):
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
   */

  if (!_initialized) {
    throw BaseException{
        "failed to apply cross-correlation filter: not initialized"};
  }

  std::feclearexcept(FE_ALL_EXCEPT);

  const auto n{_buffer.capacity()};
  const TData *samplesTemplateWf{
      TypedArray<TData>::ConstCast(_templateWaveform->data())->typedData()};
  // cross-correlation loop
  for (size_t i = 0; i < nData; ++i) {
    const TData newSample{data[i]};
    const TData lastSample{_buffer.front()};
    _sumData += newSample - lastSample;
    _sumSquaredData += util::square(newSample) - util::square(lastSample);
    const double denominatorData{
        std::sqrt(n * _sumSquaredData - _sumData * _sumData)};

    _buffer.push_back(newSample);

    double sumTemplateData{0};
    for (size_t k = 0; k < n; ++k) {
      sumTemplateData += samplesTemplateWf[k] * _buffer[k];
    }

    const double pearsonCoeff{
        (n * sumTemplateData - _sumTemplateWaveform * _sumData) /
        (_denominatorTemplateWaveform * denominatorData)};

    int fe{std::fetestexcept(FE_ALL_EXCEPT)};
    if ((fe & ~FE_INEXACT) != 0)  // we don't care about FE_INEXACT
    {
      std::vector<std::string> exceptions;
      if (fe & FE_DIVBYZERO) exceptions.push_back("FE_DIVBYZERO");
      if (fe & FE_INVALID) exceptions.push_back("FE_INVALID");
      if (fe & FE_OVERFLOW) exceptions.push_back("FE_OVERFLOW");
      if (fe & FE_UNDERFLOW) exceptions.push_back("FE_UNDERFLOW");

      std::string msg{
          "Floating point exception during cross-correlation (sample_idx=" +
          std::to_string(i) + ", sample=" + std::to_string(newSample) + "): "};
      msg += boost::algorithm::join(exceptions, ", ");
      SCDETECT_LOG_WARNING(msg.c_str());

      std::feclearexcept(FE_ALL_EXCEPT);
    }

    data[i] =
        static_cast<TData>(std::isfinite(pearsonCoeff) ? pearsonCoeff : 0);
  }
}

template <typename TData>
void CrossCorrelation<TData>::setupFilter(double samplingFrequency) {
  if (samplingFrequency <= 0) {
    return;
  }

  reset();
}

/* ------------------------------------------------------------------------ */
template <typename TData>
AdaptiveCrossCorrelation<TData>::AdaptiveCrossCorrelation(
    const GenericRecordCPtr &waveform, const std::string filterId,
    const Core::Time &templateStartTime, const Core::Time &templateEndTime,
    double samplingFrequency)
    : _wf{waveform},
      _filterId{filterId},
      _templateStartTime{templateStartTime},
      _templateEndTime{templateEndTime} {
  this->setSamplingFrequency(samplingFrequency);
}

template <typename TData>
void AdaptiveCrossCorrelation<TData>::setupFilter(double samplingFrequency) {
  this->_initialized = false;
  if (samplingFrequency <= 0) {
    return;
  }

  if (this->_samplingFrequency &&
      this->_samplingFrequency == samplingFrequency) {
    this->reset();
    this->_initialized = true;
  } else if (!this->_samplingFrequency ||
             this->_samplingFrequency != samplingFrequency) {
    this->_samplingFrequency = samplingFrequency;
    createTemplateWaveform(*this->_samplingFrequency);
    this->reset();
    this->_initialized = true;
  }
}

template <typename TData>
void AdaptiveCrossCorrelation<TData>::createTemplateWaveform(
    double targetFrequency) {
  // XXX(damb): Assume, the data is demeaned, already.
  auto wf{util::make_smart<GenericRecord>(*_wf)};

  // resample
  if (wf->samplingFrequency() != targetFrequency) {
    if (!waveform::resample(*wf, targetFrequency)) {
      throw BaseException{
          Core::stringify("failed to resample template waveform "
                          "(samplingFrequency=%f): targetFrequency=%f",
                          _wf->samplingFrequency(), targetFrequency)};
    }
    SCDETECT_LOG_DEBUG(
        "Resampled template waveform (samplingFrequency=%f): "
        "targetFrequency=%f",
        _wf->samplingFrequency(), targetFrequency);
  }
  // filter
  if (!_filterId.empty()) {
    if (!waveform::filter(*wf, _filterId)) {
      throw BaseException{
          Core::stringify("failed to filter template waveform: filter=%s,"
                          "start=%s, end=%s",
                          _filterId.c_str(), wf->startTime().iso().c_str(),
                          wf->endTime().iso().c_str())};
    }
    SCDETECT_LOG_DEBUG(
        "Filtered template waveform (samplingFrequency=%f): "
        "filter_id=%s",
        wf->samplingFrequency(), _filterId.c_str());
  }
  // trim
  Core::TimeWindow tw{_templateStartTime, _templateEndTime};
  if (!waveform::trim(*wf, tw)) {
    throw BaseException{Core::stringify(
        "failed to trim template waveform (wfStart=%s, wfEnd=%s): "
        "start=%s, end=%s",
        wf->startTime().iso().c_str(), wf->endTime().iso().c_str(),
        _templateStartTime.iso().c_str(), _templateEndTime.iso().c_str())};
  }

  this->_templateWaveform = wf;
}

}  // namespace filter
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_FILTER_CROSSCORRELATION_IPP_
