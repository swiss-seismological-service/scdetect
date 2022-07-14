#include "factory.h"

#include <seiscomp/core/timewindow.h>
#include <seiscomp/datamodel/pick.h>
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
    const binding::Bindings &bindings, const DataModel::OriginCPtr &origin,
    const std::string &sensorLocationStreamId,
    const std::vector<SensorLocationDetectionInfo::Pick> &pickInfos,
    const TimeInfo &timeInfo,
    const SensorLocationStreamConfigs &sensorLocationStreamConfigs,
    const AmplitudeProcessorConfig &amplitudeProcessorConfig) {
  // XXX(damb): no dispatch regarding horizontal components, here. Instead, the
  // `pickInfo` is taken as it is.
  assert(origin);
  assert((pickInfos.size() == sensorLocationStreamConfigs.size()));

  logging::TaggedMessage msg{sensorLocationStreamId};

  // dispatch and create ratio amplitude processors
  auto baseId{amplitudeProcessorConfig.id + settings::kProcessorIdSep +
              util::createUUID()};

  std::vector<CombiningAmplitudeProcessor::AmplitudeProcessor> underlying;
  for (const auto &pickInfo : pickInfos) {
    const auto &authorativeWaveformStreamId{
        pickInfo.authorativeWaveformStreamId};

    // dispatch and create rms amplitude processors
    auto baseId{amplitudeProcessorConfig.id + settings::kProcessorIdSep +
                util::createUUID()};
    try {
      underlying.emplace_back(CombiningAmplitudeProcessor::AmplitudeProcessor{
          {authorativeWaveformStreamId},
          detail::createRMSAmplitude(
              bindings, origin, pickInfo, timeInfo, amplitudeProcessorConfig,
              sensorLocationStreamConfigs.at(authorativeWaveformStreamId),
              baseId)});
    } catch (const std::out_of_range &) {
      continue;
    }
  }

  if (underlying.empty()) {
    msg.setText("failed to initialize underlying amplitude processors");
    throw Factory::BaseException{logging::to_string(msg)};
  }

  auto ret{util::make_unique<amplitude::MLx>(std::move(underlying))};
  ret->computeTimeWindow();
  ret->setId(baseId);

  std::vector<DataModel::PickCPtr> picks;
  std::transform(
      std::begin(pickInfos), std::end(pickInfos), std::back_inserter(picks),
      [](const SensorLocationDetectionInfo::Pick &p) { return p.pick; });
  ret->setEnvironment(origin, nullptr, picks);

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
    const binding::Bindings &bindings, const DataModel::OriginCPtr &origin,
    const SensorLocationDetectionInfo::Pick &pickInfo, const TimeInfo &timeInfo,
    const AmplitudeProcessorConfig &amplitudeProcessorConfig,
    const processing::StreamConfig &streamConfig,
    const std::string &baseProcessorId) {
  assert(origin);

  const auto sensorLocationStreamId{util::getSensorLocationStreamId(
      pickInfo.authorativeWaveformStreamId, true)};
  std::vector<std::string> sensorLocationStreamIdTokens;
  util::tokenizeWaveformStreamId(sensorLocationStreamId,
                                 sensorLocationStreamIdTokens);
  const auto &sensorLocationBindings{factory::detail::loadSensorLocationConfig(
      bindings, sensorLocationStreamIdTokens[0],
      sensorLocationStreamIdTokens[1], sensorLocationStreamIdTokens[2],
      sensorLocationStreamIdTokens[3])};
  const auto &amplitudeProcessingConfig{
      sensorLocationBindings.amplitudeProcessingConfig};

  logging::TaggedMessage msg{sensorLocationStreamId};

  auto ret{util::make_unique<RMSAmplitude>(
      RMSAmplitude::SignalTimeInfo{timeInfo.leading, timeInfo.trailing})};
  ret->setId(baseProcessorId + settings::kProcessorIdSep + util::createUUID());

  // XXX(damb): do not provide a sensor location (currently not required)
  ret->setEnvironment(origin, nullptr, {pickInfo.pick});

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
                  "\", init_time=" + std::to_string(initTime));
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
    const binding::Bindings &bindings,
    const factory::SensorLocationDetectionInfo &sensorLocationDetectionInfo,
    const detector::Detector &detector) {
  assert(sensorLocationDetectionInfo.origin);
  assert(!sensorLocationDetectionInfo.pickMap.empty());

  std::vector<CombiningAmplitudeProcessor::AmplitudeProcessor> underlying;
  // dispatch and create ratio amplitude processors
  auto baseId{detector.id() + settings::kProcessorIdSep + util::createUUID()};
  std::vector<DataModel::PickCPtr> picks;
  for (const auto &pickMapPair : sensorLocationDetectionInfo.pickMap) {
    picks.push_back(pickMapPair.second.pick);

    // create a pseudo detection info
    auto sensorLocationDetectionInfoCopy{sensorLocationDetectionInfo};
    sensorLocationDetectionInfoCopy.pickMap.clear();
    sensorLocationDetectionInfoCopy.pickMap.emplace(pickMapPair);

    const auto &waveformStreamId{
        pickMapPair.second.authorativeWaveformStreamId};

    underlying.emplace_back(CombiningAmplitudeProcessor::AmplitudeProcessor{
        {waveformStreamId},
        Factory::createRatioAmplitude(bindings, sensorLocationDetectionInfoCopy,
                                      detector, baseId)});
  }

  auto ret{util::make_unique<MRelative>(std::move(underlying))};
  ret->computeTimeWindow();
  ret->setId(baseId);
  ret->setEnvironment(sensorLocationDetectionInfo.origin, nullptr, picks);

  return ret;
}

std::unique_ptr<AmplitudeProcessor> Factory::createMLx(
    const binding::Bindings &bindings,
    const factory::SensorLocationDetectionInfo &sensorLocationDetectionInfo,
    const detector::Detector &detector) {
  assert(sensorLocationDetectionInfo.origin);
  assert(!sensorLocationDetectionInfo.pickMap.empty());

  // XXX(damb): pick is randomly chosen; use the earliest one
  DataModel::PickCPtr earliest;
  {
    std::vector<factory::SensorLocationDetectionInfo::Pick> pickInfos;
    std::transform(
        std::begin(sensorLocationDetectionInfo.pickMap),
        std::end(sensorLocationDetectionInfo.pickMap),
        std::back_inserter(pickInfos),
        [](const factory::SensorLocationDetectionInfo::PickMap::value_type &p) {
          return p.second;
        });

    std::sort(std::begin(pickInfos), std::end(pickInfos),
              [](const decltype(pickInfos)::value_type &lhs,
                 const decltype(pickInfos)::value_type &rhs) {
                return lhs.pick->time().value() < rhs.pick->time().value();
              });
    earliest = pickInfos.front().pick;
  }

  std::vector<factory::SensorLocationDetectionInfo::Pick> pickInfos;

  factory::SensorLocationStreamConfigs sensorLocationStreamConfigs;

  std::vector<std::string> sensorLocationStreamIdTokens;
  util::tokenizeWaveformStreamId(
      sensorLocationDetectionInfo.sensorLocationStreamId,
      sensorLocationStreamIdTokens);
  try {
    const util::HorizontalComponents horizontalComponents{
        Client::Inventory::Instance(),   sensorLocationStreamIdTokens[0],
        sensorLocationStreamIdTokens[1], sensorLocationStreamIdTokens[2],
        sensorLocationStreamIdTokens[3], earliest->time().value()};
    for (const auto &s : horizontalComponents) {
      processing::StreamConfig streamConfig;
      streamConfig.init(s);

      auto authorativeWaveformStreamId{util::join(
          sensorLocationStreamIdTokens[0], sensorLocationStreamIdTokens[1],
          sensorLocationStreamIdTokens[2], s->code())};
      sensorLocationStreamConfigs.emplace(authorativeWaveformStreamId,
                                          streamConfig);

      pickInfos.push_back({authorativeWaveformStreamId, earliest});
    }
  } catch (const Exception &e) {
    logging::TaggedMessage msg{
        sensorLocationDetectionInfo.sensorLocationStreamId,
        "failed to load stream configuration"};
    throw Factory::BaseException{logging::to_string(msg)};
  }

  factory::TimeInfo timeInfo;
  for (const auto &pickMapPair : sensorLocationDetectionInfo.pickMap) {
    const auto &pickInfo{pickMapPair.second};
    if (pickInfo.pick != earliest) {
      continue;
    }

    const auto &templateWaveformProcessorId{pickMapPair.first};
    const auto &templateWaveformProcessor{
        detector.processor(templateWaveformProcessorId)};
    assert(templateWaveformProcessor);
    const auto &templateWaveform{templateWaveformProcessor->templateWaveform()};
    assert(templateWaveform.referenceTime());

    timeInfo.leading = templateWaveform.configuredStartTime() -
                       *templateWaveform.referenceTime();
    timeInfo.trailing = templateWaveform.configuredEndTime() -
                        *templateWaveform.referenceTime();
  }

  factory::AmplitudeProcessorConfig amplitudeProcessorConfig{
      detector.id(), detector.gapThreshold(), detector.gapTolerance(),
      detector.gapInterpolation()};

  return factory::createMLx(bindings, sensorLocationDetectionInfo.origin,
                            sensorLocationDetectionInfo.sensorLocationStreamId,
                            pickInfos, timeInfo, sensorLocationStreamConfigs,
                            amplitudeProcessorConfig);
}

void Factory::reset() { resetCallbacks(); }

std::unique_ptr<AmplitudeProcessor> Factory::createRatioAmplitude(
    const binding::Bindings &bindings,
    const factory::SensorLocationDetectionInfo &sensorLocationDetectionInfo,
    const detector::Detector &detector, const std::string &baseProcessorId) {
  assert(sensorLocationDetectionInfo.origin);
  assert((sensorLocationDetectionInfo.pickMap.size() == 1));

  std::vector<std::string> sensorLocationStreamIdTokens;
  util::tokenizeWaveformStreamId(
      sensorLocationDetectionInfo.sensorLocationStreamId,
      sensorLocationStreamIdTokens);
  const auto &sensorLocationBindings{factory::detail::loadSensorLocationConfig(
      bindings, sensorLocationStreamIdTokens[0],
      sensorLocationStreamIdTokens[1], sensorLocationStreamIdTokens[2],
      sensorLocationStreamIdTokens[3])};
  const auto &amplitudeProcessingConfig{
      sensorLocationBindings.amplitudeProcessingConfig};

  logging::TaggedMessage msg{
      sensorLocationDetectionInfo.sensorLocationStreamId};

  auto ret{util::make_unique<RatioAmplitude>()};
  ret->setId(baseProcessorId + settings::kProcessorIdSep + util::createUUID());

  const auto &pickPair{*std::begin(sensorLocationDetectionInfo.pickMap)};
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
  ret->setEnvironment(sensorLocationDetectionInfo.origin, nullptr, {pick.pick});

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
