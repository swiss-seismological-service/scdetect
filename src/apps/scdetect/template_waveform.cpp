#include "template_waveform.h"

#include <seiscomp/core/timewindow.h>

#include <boost/variant2/variant.hpp>
#include <cassert>
#include <exception>
#include <string>
#include <type_traits>

#include "exception.h"
#include "util/filter.h"
#include "util/memory.h"
#include "waveform.h"

namespace Seiscomp {
namespace detect {

TemplateWaveform::ProcessingConfig::ProcessingConfig(
    const ProcessingConfig &other)
    : templateStartTime{other.templateStartTime},
      templateEndTime{other.templateEndTime},
      initTime{other.initTime},
      safetyMargin{other.safetyMargin},
      samplingFrequency{other.samplingFrequency},
      detrend{other.detrend},
      demean{other.demean} {
  try {
    filter = boost::variant2::get<1>(other.filter);
  } catch (const boost::variant2::bad_variant_access &) {
    const auto *ptr{boost::variant2::get<0>(other.filter).get()};
    if (static_cast<bool>(ptr)) {
      std::unique_ptr<DoubleFilter> cloned(ptr->clone());
      filter = std::move(cloned);
    } else {
      filter = nullptr;
    }
  }
}

TemplateWaveform::ProcessingConfig &
TemplateWaveform::ProcessingConfig::operator=(ProcessingConfig other) noexcept {
  std::swap(templateStartTime, other.templateStartTime);
  std::swap(templateEndTime, other.templateEndTime);
  std::swap(filter, other.filter);
  std::swap(initTime, other.initTime);
  std::swap(safetyMargin, other.safetyMargin);
  std::swap(samplingFrequency, other.samplingFrequency);
  std::swap(detrend, other.detrend);
  std::swap(demean, other.demean);
  return *this;
}

const TemplateWaveform::ProcessingStrategy TemplateWaveform::noProcessing =
    [](const GenericRecordCPtr &raw,
       const TemplateWaveform::ProcessingConfig &config) -> GenericRecordCPtr {
  return raw;
};

const TemplateWaveform::ProcessingStrategy TemplateWaveform::defaultProcessing =
    [](const GenericRecordCPtr &raw,
       const TemplateWaveform::ProcessingConfig &config) -> GenericRecordCPtr {
  // XXX(damb): Assume, the data is demeaned, already.
  auto ret{util::make_smart<GenericRecord>(*raw)};
  // the caller is required to copy the data
  // https://github.com/SeisComP/common/issues/38
  ret->setData(dynamic_cast<DoubleArray *>(raw->data()->copy(Array::DOUBLE)));

  if (config.detrend) {
    waveform::detrend(*ret);
  }

  if (config.demean) {
    waveform::demean(*ret);
  }

  // resample
  if (config.samplingFrequency &&
      ret->samplingFrequency() != *config.samplingFrequency) {
    if (!waveform::resample(*ret, *config.samplingFrequency)) {
      throw Exception{
          "failed to resample template waveform (sampling_frequency=" +
          std::to_string(raw->samplingFrequency()) +
          ") target_sampling_frequency=" +
          std::to_string(*config.samplingFrequency)};
    }
  }
  // filter
  try {
    auto *filter{boost::variant2::get<0>(config.filter).get()};
    if (!waveform::filter(*ret, filter)) {
      throw Exception{"failed to filter template waveform"};
    }
  } catch (const boost::variant2::bad_variant_access &) {
    auto filter{boost::variant2::get<1>(config.filter)};
    if (filter && !filter.value().empty()) {
      if (!waveform::filter(*ret, *filter)) {
        throw Exception{"failed to filter template waveform: filter=" +
                        *filter};
      }
    }
  }

  // trim
  Core::TimeWindow tw{config.templateStartTime.value_or(raw->startTime()),
                      config.templateEndTime.value_or(raw->endTime())};
  if (!waveform::trim(*ret, tw)) {
    throw Exception{
        "failed to trim template waveform (raw_start=" +
        raw->startTime().iso() + ", raw_end=" + raw->endTime().iso() +
        "): start=" + tw.startTime().iso() + ", end=" + tw.endTime().iso()};
  }

  return ret;
};

TemplateWaveform::TemplateWaveform(const GenericRecordCPtr &waveform)
    : _raw{waveform} {}

TemplateWaveform::TemplateWaveform(const GenericRecordCPtr &waveform,
                                   const ProcessingConfig &processingConfig,
                                   const ProcessingStrategy &processingStrategy)
    : _processingConfig{processingConfig},
      _processingStrategy{processingStrategy},
      _raw{waveform} {}

TemplateWaveform TemplateWaveform::load(
    WaveformHandlerIface *waveformHandler, const std::string &netCode,
    const std::string &staCode, const std::string &locCode,
    const std::string &chaCode, const ProcessingConfig &processingConfig,
    const ProcessingStrategy &processingStrategy) {
  assert(
      (processingConfig.templateStartTime && processingConfig.templateEndTime));

  WaveformHandler::ProcessingConfig config;
  config.demean = false;

  auto leadingMargin{
      processingConfig.safetyMargin.value_or(Core::TimeSpan{0.0})};

  bool hasFilter{false};
  try {
    auto filter{boost::variant2::get<1>(processingConfig.filter)};
    hasFilter = filter && !filter.value().empty();
  } catch (const boost::variant2::bad_variant_access &) {
    auto *filter{boost::variant2::get<0>(processingConfig.filter).get()};
    hasFilter = static_cast<bool>(filter);
  }

  if (hasFilter) {
    leadingMargin = std::max(
        leadingMargin, processingConfig.initTime.value_or(Core::TimeSpan{0.0}));
  }

  const Core::TimeWindow tw{
      processingConfig.templateStartTime.value() - leadingMargin,
      processingConfig.templateEndTime.value() +
          processingConfig.safetyMargin.value_or(Core::TimeSpan{0.0})};

  GenericRecordCPtr raw;
  try {
    raw = waveformHandler->get(netCode, staCode, locCode, chaCode, tw, config);
  } catch (std::exception &e) {
    throw WaveformHandler::NoData{e.what()};
  } catch (...) {
    throw WaveformHandler::NoData{};
  }

  return TemplateWaveform{raw, processingConfig, processingStrategy};
}

const GenericRecord &TemplateWaveform::raw() const { return *_raw; }

void TemplateWaveform::setRaw(const GenericRecordCPtr &raw) {
  reset();
  _raw = raw;
}

const GenericRecord &TemplateWaveform::waveform() const {
  return templateWaveform();
}

const TemplateWaveform::ProcessingConfig &TemplateWaveform::processingConfig()
    const {
  return _processingConfig;
}

std::string TemplateWaveform::waveformStreamId() const {
  assert(_raw);
  return _raw->streamID();
}

void TemplateWaveform::setProcessingConfig(const ProcessingConfig &config) {
  reset();
  _processingConfig = config;
}

void TemplateWaveform::setProcessingStrategy(
    const ProcessingStrategy &strategy) {
  reset();
  _processingStrategy = strategy;
}

void TemplateWaveform::setSamplingFrequency(double samplingFrequency) {
  if (samplingFrequency <= 0) {
    return;
  }
  if (samplingFrequency != this->samplingFrequency()) {
    reset();
  }
  _processingConfig.samplingFrequency = samplingFrequency;
}

double TemplateWaveform::samplingFrequency() const {
  assert(_raw);
  return _processingConfig.samplingFrequency.value_or(
      _raw->samplingFrequency());
}

std::size_t TemplateWaveform::size() const {
  return templateWaveform().sampleCount();
}

Core::TimeSpan TemplateWaveform::length() const {
  return Core::TimeSpan{templateWaveform().timeWindow().length()};
}

Core::Time TemplateWaveform::startTime() const {
  return templateWaveform().startTime();
}

Core::Time TemplateWaveform::endTime() const {
  return templateWaveform().endTime();
}

Core::Time TemplateWaveform::configuredStartTime() const {
  assert(_raw);
  return _processingConfig.templateStartTime.value_or(_raw->startTime());
}

Core::Time TemplateWaveform::configuredEndTime() const {
  assert(_raw);
  return _processingConfig.templateEndTime.value_or(_raw->endTime());
}

void TemplateWaveform::setReferenceTime(const boost::optional<Core::Time> &t) {
  assert((!t || (t >= configuredStartTime() && t <= configuredEndTime())));
  _referenceTime = t;
}

// Returns the (optional) template waveform reference time
const boost::optional<Core::Time> &TemplateWaveform::referenceTime() const {
  return _referenceTime;
}

void TemplateWaveform::reset() {
  _templateWaveform.reset();
  // reset the filter state
  try {
    util::reset(boost::variant2::get<0>(_processingConfig.filter));
  } catch (const boost::variant2::bad_variant_access &) {
  }
}

const GenericRecord &TemplateWaveform::templateWaveform() const {
  if (!_templateWaveform) {
    assert(_raw);
    _templateWaveform = _processingStrategy(_raw, _processingConfig);
  }
  return *_templateWaveform;
}

}  // namespace detect
}  // namespace Seiscomp
