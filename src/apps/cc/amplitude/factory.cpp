#include "factory.h"

#include <seiscomp/core/timewindow.h>
#include <seiscomp/processing/stream.h>

#include <algorithm>
#include <cassert>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "../def.h"
#include "../log.h"
#include "../settings.h"
#include "../util/horizontal_components.h"
#include "../util/memory.h"
#include "../util/util.h"
#include "../util/waveform_stream_id.h"
#include "mlx.h"
#include "mrelative.h"
#include "ratio.h"
#include "rms.h"

namespace Seiscomp {
namespace detect {
namespace amplitude {
namespace factory {

std::unique_ptr<amplitude::MLx> createMLx(
    const binding::Bindings &bindings, const Detection &detection,
    const SensorLocationTimeInfo &sensorLocationTimeInfo,
    const AmplitudeProcessorConfig &amplitudeProcessorConfig) {
  assert(detection.origin);
  assert(!detection.pickMap.empty());

  std::vector<std::string> sensorLocationStreamIdTokens;
  util::tokenizeWaveformStreamId(detection.sensorLocationStreamId,
                                 sensorLocationStreamIdTokens);
  const auto &sensorLocationBindings{factory::detail::loadSensorLocationConfig(
      bindings, sensorLocationStreamIdTokens[0],
      sensorLocationStreamIdTokens[1], sensorLocationStreamIdTokens[2],
      sensorLocationStreamIdTokens[3])};

  std::vector<DataModel::PickCPtr> picks;
  std::transform(std::begin(detection.pickMap), std::end(detection.pickMap),
                 std::back_inserter(picks),
                 [](const decltype(detection.pickMap)::value_type &p) {
                   return p.second.pick;
                 });
  // choose arbitrarily the earliest pick w.r.t. a sensor location
  std::sort(std::begin(picks), std::end(picks),
            [](const decltype(picks)::value_type &lhs,
               const decltype(picks)::value_type &rhs) {
              return lhs->time().value() < rhs->time().value();
            });
  auto &earliestPick{picks.front()};

  logging::TaggedMessage msg{detection.sensorLocationStreamId};

  std::vector<CombiningAmplitudeProcessor::AmplitudeProcessor> underlying;
  // dispatch and create ratio amplitude processors
  auto baseId{amplitudeProcessorConfig.id + settings::kProcessorIdSep +
              util::createUUID()};

  // index detection info
  std::unordered_map<Detection::WaveformStreamId,
                     Detection::PickMap::const_iterator>
      detectionInfoIdx;
  for (auto cit{detection.pickMap.cbegin()}; cit != detection.pickMap.cend();
       ++cit) {
    detectionInfoIdx.emplace(cit->second.authorativeWaveformStreamId, cit);
  }

  try {
    const util::HorizontalComponents horizontalComponents{
        Client::Inventory::Instance(),   sensorLocationStreamIdTokens[0],
        sensorLocationStreamIdTokens[1], sensorLocationStreamIdTokens[2],
        sensorLocationStreamIdTokens[3], earliestPick->time().value()};
    for (const auto &s : horizontalComponents) {
      processing::StreamConfig streamConfig;
      streamConfig.init(s);

      auto waveformStreamId{util::join(
          horizontalComponents.netCode(), horizontalComponents.staCode(),
          horizontalComponents.locCode(), s->code())};

      // create pseudo detection
      auto detectionCopy{detection};
      detectionCopy.pickMap.clear();
      try {
        const auto &detectionInfo{detectionInfoIdx.at(waveformStreamId)};
        detectionCopy.pickMap.emplace(detectionInfo->first,
                                      detectionInfo->second);
      } catch (const std::out_of_range &e) {
        continue;
      }

      Core::TimeWindow tw;
      try {
        const auto &detectionInfo{detectionInfoIdx.at(waveformStreamId)};
        const auto &referenceTime{detectionInfo->second.pick->time().value()};
        const auto &timeInfoConfig{
            sensorLocationTimeInfo.timeInfos.at(waveformStreamId)};
        tw.setStartTime(referenceTime - timeInfoConfig.leading);
        tw.setEndTime(referenceTime + timeInfoConfig.trailing);
      } catch (std::out_of_range &e) {
        continue;
      }

      underlying.emplace_back(CombiningAmplitudeProcessor::AmplitudeProcessor{
          {waveformStreamId},
          detail::createRMSAmplitude(bindings, detectionCopy, tw,
                                     amplitudeProcessorConfig, streamConfig,
                                     baseId)});
    }
  } catch (Exception &e) {
    msg.setText(std::string{e.what()} +
                " (pick_time=" + earliestPick->time().value().iso() + ")");
    throw Factory::BaseException{logging::to_string(msg)};
  }

  if (underlying.empty()) {
    msg.setText("failed to initialize underlying amplitude processors");
    throw Factory::BaseException{logging::to_string(msg)};
  }

  auto ret{util::make_unique<amplitude::MLx>(std::move(underlying))};
  ret->computeTimeWindow();
  ret->setId(baseId);
  ret->setEnvironment(detection.origin, nullptr, picks);

  return ret;
}

namespace detail {

const binding::SensorLocationConfig &loadSensorLocationConfig(
    const binding::Bindings &bindings, const std::string &netCode,
    const std::string &staCode, const std::string &locCode,
    const std::string &chaCode) {
  try {
    return bindings.at(netCode, staCode, locCode, chaCode);
  } catch (std::out_of_range &e) {
    logging::TaggedMessage msg{
        util::join(netCode, staCode, locCode, chaCode),
        "failed to load bindings configuration: " + std::string{e.what()}};
    throw Factory::BaseException{logging::to_string(msg)};
  }
}

std::unique_ptr<RMSAmplitude> createRMSAmplitude(
    const binding::Bindings &bindings, const factory::Detection &detection,
    const Core::TimeWindow &timeWindow,
    const AmplitudeProcessorConfig &amplitudeProcessorConfig,
    const processing::StreamConfig &streamConfig,
    const std::string &baseProcessorId) {
  assert(detection.origin);
  assert((detection.pickMap.size() == 1));

  std::vector<std::string> sensorLocationStreamIdTokens;
  util::tokenizeWaveformStreamId(detection.sensorLocationStreamId,
                                 sensorLocationStreamIdTokens);
  const auto &sensorLocationBindings{factory::detail::loadSensorLocationConfig(
      bindings, sensorLocationStreamIdTokens[0],
      sensorLocationStreamIdTokens[1], sensorLocationStreamIdTokens[2],
      sensorLocationStreamIdTokens[3])};
  const auto &amplitudeProcessingConfig{
      sensorLocationBindings.amplitudeProcessingConfig};

  logging::TaggedMessage msg{detection.sensorLocationStreamId};

  const auto &pickPair{*std::begin(detection.pickMap)};
  const auto &processorId{pickPair.first};
  const auto &pick{pickPair.second};

  auto ret{util::make_unique<RMSAmplitude>()};
  ret->setId(baseProcessorId + settings::kProcessorIdSep + util::createUUID());

  // XXX(damb): do not provide a sensor location (currently not required)
  ret->setEnvironment(detection.origin, nullptr, {pick.pick});

  ret->setStreamConfig(streamConfig);
  // configure deconvolution configuration
  AmplitudeProcessor::DeconvolutionConfig deconvolutionConfig;
  try {
    deconvolutionConfig = static_cast<AmplitudeProcessor::DeconvolutionConfig>(
        sensorLocationBindings.at(streamConfig.code()).deconvolutionConfig);
  } catch (std::out_of_range &e) {
    binding::StreamConfig::DeconvolutionConfig fallback;
    msg.setText(
        "failed to look up deconvolution configuration related bindings "
        "(channel code: \"" +
        streamConfig.code() +
        "\") required for amplitude processor configuration (" + e.what() +
        "); using fallback configuration, instead: \"" +
        fallback.debugString() + "\"");
    SCDETECT_LOG_WARNING_TAGGED(ret->id(), "%s",
                                logging::to_string(msg).c_str());
    deconvolutionConfig =
        static_cast<AmplitudeProcessor::DeconvolutionConfig>(fallback);
  }

  // configure filter
  auto filter{amplitudeProcessingConfig.mlx.filter};
  bool filterConfigured{false};
  if (filter) {
    util::replaceEscapedXMLFilterIdChars(*filter);
    if (!filter.value().empty()) {
      auto initTime = amplitudeProcessingConfig.mlx.initTime;
      msg.setText("Configured amplitude processor filter: filter=\"" + *filter +
                  "init_time=" + std::to_string(initTime));
      SCDETECT_LOG_DEBUG_TAGGED(ret->id(), "%s",
                                logging::to_string(msg).c_str());

      ret->setFilter(processing::createFilter(*filter), initTime);
      filterConfigured = true;
    }
  }
  if (!filterConfigured) {
    msg.setText("Configured amplitude processor without filter: filter=\"\"");
    SCDETECT_LOG_DEBUG_TAGGED(ret->id(), "%s", logging::to_string(msg).c_str());
  }

  ret->setTimeWindow(timeWindow);
  if (amplitudeProcessorConfig.gapInterpolation) {
    ret->setGapInterpolation(amplitudeProcessorConfig.gapInterpolation);
    ret->setGapThreshold(amplitudeProcessorConfig.gapThreshold);
    ret->setGapTolerance(amplitudeProcessorConfig.gapTolerance);
  }

  ret->setSaturationThreshold(
      amplitudeProcessingConfig.mlx.saturationThreshold);

  return ret;
}

}  // namespace detail
}  // namespace factory

Factory::BaseException::BaseException()
    : BaseException{"base factory exception"} {}

std::unique_ptr<detect::AmplitudeProcessor> Factory::createMRelative(
    const binding::Bindings &bindings, const factory::Detection &detection,
    const detector::Detector &detector) {
  assert(detection.origin);
  assert(!detection.pickMap.empty());

  std::vector<CombiningAmplitudeProcessor::AmplitudeProcessor> underlying;
  // dispatch and create ratio amplitude processors
  auto baseId{detector.id() + settings::kProcessorIdSep + util::createUUID()};
  std::vector<DataModel::PickCPtr> picks;
  for (const auto &pickMapPair : detection.pickMap) {
    picks.push_back(pickMapPair.second.pick);

    // create a pseudo detection
    auto detectionCopy{detection};
    detectionCopy.pickMap.clear();
    detectionCopy.pickMap.emplace(pickMapPair);

    const auto &waveformStreamId{
        pickMapPair.second.authorativeWaveformStreamId};

    underlying.emplace_back(CombiningAmplitudeProcessor::AmplitudeProcessor{
        {waveformStreamId},
        Factory::createRatioAmplitude(bindings, detectionCopy, detector,
                                      baseId)});
  }

  auto ret{util::make_unique<MRelative>(std::move(underlying))};
  ret->computeTimeWindow();
  ret->setId(baseId);
  ret->setEnvironment(detection.origin, nullptr, picks);

  return ret;
}

std::unique_ptr<AmplitudeProcessor> Factory::createMLx(
    const binding::Bindings &bindings, const factory::Detection &detection,
    const detector::Detector &detector) {
  assert(detection.origin);
  assert(!detection.pickMap.empty());

  factory::AmplitudeProcessorConfig amplitudeProcessorConfig{
      detector.id(), detector.gapThreshold(), detector.gapTolerance(),
      detector.gapInterpolation()};

  factory::SensorLocationTimeInfo sensorLocationTimeInfo;
  for (const auto &pickMapPair : detection.pickMap) {
    const auto &templateWaveformProcessorId{pickMapPair.first};
    const auto &templateWaveformProcessor{
        detector.processor(templateWaveformProcessorId)};
    assert(templateWaveformProcessor);
    const auto &templateWaveform{templateWaveformProcessor->templateWaveform()};
    const auto &pickInfo{pickMapPair.second};
    const auto referenceTime{pickInfo.pick->time().value()};
    assert(templateWaveform.referenceTime());

    sensorLocationTimeInfo.timeInfos.emplace(
        pickInfo.authorativeWaveformStreamId,
        factory::SensorLocationTimeInfo::TimeInfo{
            *templateWaveform.referenceTime() -
                templateWaveform.configuredStartTime(),
            *templateWaveform.referenceTime() +
                templateWaveform.configuredEndTime()});
  }

  return factory::createMLx(bindings, detection, sensorLocationTimeInfo,
                            amplitudeProcessorConfig);
}

void Factory::reset() { resetCallbacks(); }

std::unique_ptr<AmplitudeProcessor> Factory::createRatioAmplitude(
    const binding::Bindings &bindings, const factory::Detection &detection,
    const detector::Detector &detector, const std::string &baseProcessorId) {
  assert(detection.origin);
  assert((detection.pickMap.size() == 1));

  std::vector<std::string> sensorLocationStreamIdTokens;
  util::tokenizeWaveformStreamId(detection.sensorLocationStreamId,
                                 sensorLocationStreamIdTokens);
  const auto &sensorLocationBindings{factory::detail::loadSensorLocationConfig(
      bindings, sensorLocationStreamIdTokens[0],
      sensorLocationStreamIdTokens[1], sensorLocationStreamIdTokens[2],
      sensorLocationStreamIdTokens[3])};
  const auto &amplitudeProcessingConfig{
      sensorLocationBindings.amplitudeProcessingConfig};

  logging::TaggedMessage msg{detection.sensorLocationStreamId};

  auto ret{util::make_unique<RatioAmplitude>()};
  ret->setId(baseProcessorId + settings::kProcessorIdSep + util::createUUID());

  const auto &pickPair{*std::begin(detection.pickMap)};
  const auto &processorId{pickPair.first};
  const auto &pick{pickPair.second};

  const auto *templateWaveformProcessor{detector.processor(processorId)};
  assert(templateWaveformProcessor);
  auto templateWaveform{templateWaveformProcessor->templateWaveform()};

  auto filter{amplitudeProcessingConfig.mrelative.filter};
  bool templateWaveformHasFilterConfigured{false};
  if (filter) {
    util::replaceEscapedXMLFilterIdChars(*filter);
    if (!filter.value().empty()) {
      auto processingConfig{templateWaveform.processingConfig()};
      processingConfig.filter = filter;
      processingConfig.initTime = amplitudeProcessingConfig.mrelative.initTime;
      templateWaveform.setProcessingConfig(processingConfig);
      msg.setText("Configured amplitude processor filter: filter=\"" + *filter +
                  "init_time=" +
                  std::to_string(amplitudeProcessingConfig.mrelative.initTime));
      SCDETECT_LOG_DEBUG_TAGGED(ret->id(), "%s",
                                logging::to_string(msg).c_str());

      templateWaveformHasFilterConfigured = true;
    }
  } else {
    // use the filter from detection waveform processing as a fallback
    auto processingConfig{templateWaveform.processingConfig()};
    auto *templateWaveformProcessorFilter{templateWaveformProcessor->filter()};
    if (static_cast<bool>(templateWaveformProcessorFilter)) {
      std::unique_ptr<DoubleFilter> cloned{
          templateWaveformProcessorFilter->clone()};
      processingConfig.filter = std::move(cloned);
      processingConfig.initTime = templateWaveformProcessor->initTime();
      msg.setText(
          "Configured amplitude processor with detection processing filter: "
          "processor_id=" +
          templateWaveformProcessor->id());
      SCDETECT_LOG_DEBUG_TAGGED(ret->id(), "%s",
                                logging::to_string(msg).c_str());

      templateWaveformHasFilterConfigured = true;
    }
  }

  if (!templateWaveformHasFilterConfigured) {
    msg.setText("Configured amplitude processor without filter: filter=\"\"");
    SCDETECT_LOG_DEBUG_TAGGED(ret->id(), "%s", logging::to_string(msg).c_str());
  }

  ret->setTemplateWaveform(templateWaveform);

  // XXX(damb): do not provide a sensor location (currently not required)
  ret->setEnvironment(detection.origin, nullptr, {pick.pick});

  ret->computeTimeWindow();
  if (detector.gapInterpolation()) {
    ret->setGapInterpolation(detector.gapInterpolation());
    ret->setGapThreshold(detector.gapThreshold());
    ret->setGapTolerance(detector.gapTolerance());
  }

  ret->setSaturationThreshold(
      amplitudeProcessingConfig.mrelative.saturationThreshold);

  return ret;
}

}  // namespace amplitude
}  // namespace detect
}  // namespace Seiscomp
