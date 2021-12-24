#include "template_waveform.h"

#include <string>

#include "exception.h"
#include "log.h"
#include "seiscomp/core/genericrecord.h"
#include "util/memory.h"
#include "waveform.h"

namespace Seiscomp {
namespace detect {

TemplateWaveform::TemplateWaveform(const GenericRecordCPtr &waveform)
    : _raw{waveform} {}

TemplateWaveform::TemplateWaveform(
    const GenericRecordCPtr &waveform,
    const boost::optional<Core::Time> &templateStartTime,
    const boost::optional<Core::Time> &templateEndTime,
    const boost::optional<std::string> &filter,
    boost::optional<double> samplingFrequency)
    : _raw{waveform},
      _startTime{templateStartTime},
      _endTime{templateEndTime},
      _filter{filter},
      _samplingFrequency{samplingFrequency} {}

void TemplateWaveform::setSamplingFrequency(double samplingFrequency) {
  if (samplingFrequency <= 0) {
    return;
  }
  if (samplingFrequency != this->samplingFrequency()) {
    reset();
  }
  _samplingFrequency = samplingFrequency;
}

const GenericRecord &TemplateWaveform::raw() const { return *_raw; }

const GenericRecord &TemplateWaveform::waveform() const {
  return templateWaveform();
}

double TemplateWaveform::samplingFrequency() const {
  return _samplingFrequency.value_or(_raw->samplingFrequency());
}

std::size_t TemplateWaveform::size() const {
  return templateWaveform().sampleCount();
}

double TemplateWaveform::length() const {
  return templateWaveform().timeWindow().length();
}

Core::Time TemplateWaveform::startTime() const {
  return templateWaveform().startTime();
}

Core::Time TemplateWaveform::endTime() const {
  return templateWaveform().endTime();
}

void TemplateWaveform::reset() { _templateWaveform.reset(); }

const GenericRecord &TemplateWaveform::templateWaveform() const {
  if (!_templateWaveform) {
    _templateWaveform = createTemplateWaveform(samplingFrequency());
  }
  return *_templateWaveform;
}

GenericRecordCPtr TemplateWaveform::createTemplateWaveform(
    double targetSamplingFrequency) const {
  // XXX(damb): Assume, the data is demeaned, already.
  auto ret{util::make_smart<GenericRecord>(*_raw)};

  // resample
  if (ret->samplingFrequency() != targetSamplingFrequency) {
    if (!waveform::resample(*ret, targetSamplingFrequency)) {
      throw Exception{
          "failed to resample template waveform (sampling_frequency=" +
          std::to_string(_raw->samplingFrequency()) +
          ") target_sampling_frequency=" +
          std::to_string(targetSamplingFrequency)};
    }
    SCDETECT_LOG_DEBUG(
        "Resampled template waveform (sampling_frequency=%f): "
        "targetFrequency=%f",
        _raw->samplingFrequency(), targetSamplingFrequency);
  }
  // filter
  if (_filter && !_filter.value().empty()) {
    if (!waveform::filter(*ret, *_filter)) {
      throw Exception{"failed to filter template waveform: filter=" + *_filter};
    }
    SCDETECT_LOG_DEBUG(
        "Filtered template waveform (sampling_frequency=%f): "
        "filter=%s",
        ret->samplingFrequency(), _filter.value().c_str());
  }
  // trim
  Core::TimeWindow tw{_startTime.value_or(_raw->startTime()),
                      _endTime.value_or(_raw->endTime())};
  if (!waveform::trim(*ret, tw)) {
    throw Exception{"failed to trim template waveform (raw_start=" +
                    _raw->startTime().iso() +
                    ", raw_end=" + _raw->endTime().iso() +
                    "): start=" + _startTime.value_or(_raw->startTime()).iso() +
                    ", end=" + _endTime.value_or(_raw->endTime()).iso()};
  }

  return ret;
}

}  // namespace detect
}  // namespace Seiscomp
