#include "factory.h"

#include <seiscomp/processing/stream.h>

#include <algorithm>
#include <cassert>
#include <stdexcept>
#include <string>
#include <vector>

#include "../log.h"
#include "../settings.h"
#include "../util/horizontal_components.h"
#include "../util/memory.h"
#include "../util/util.h"
#include "../util/waveform_stream_id.h"
#include "mlx.h"
#include "mrelative.h"
#include "ratio.h"

namespace Seiscomp {
namespace detect {
namespace amplitude {

Factory::BaseException::BaseException()
    : BaseException{"base factory exception"} {}

std::unique_ptr<detect::AmplitudeProcessor> Factory::createMRelative(
    const binding::Bindings &bindings, const factory::Detection &detection,
    const detector::DetectorWaveformProcessor &detector) {
  assert(detection.origin);
  assert(!detection.pickMap.empty());

  std::vector<CombiningAmplitudeProcessor::AmplitudeProcessor> underlying;
  // dispatch and create ratio amplitude processors
  for (const auto &pickMapPair : detection.pickMap) {
    auto detectionCopy{detection};
    detectionCopy.pickMap.clear();
    detectionCopy.pickMap.emplace(pickMapPair);

    const auto &waveformStreamId{
        pickMapPair.second.authorativeWaveformStreamId};
    underlying.emplace_back(CombiningAmplitudeProcessor::AmplitudeProcessor{
        {waveformStreamId},
        Factory::createRatioAmplitude(bindings, detectionCopy, detector)});
  }

  auto ret{util::make_unique<MRelative>(std::move(underlying))};
  ret->computeTimeWindow();

  return ret;
}

std::unique_ptr<AmplitudeProcessor> Factory::createMLx(
    const binding::Bindings &bindings, const factory::Detection &detection,
    const detector::DetectorWaveformProcessor &detector) {
  assert(detection.origin);
  assert(!detection.pickMap.empty());

  std::vector<DataModel::PickCPtr> picks;
  std::transform(std::begin(detection.pickMap), std::end(detection.pickMap),
                 std::back_inserter(picks),
                 [](const decltype(detection.pickMap)::value_type &p) {
                   return p.second.pick;
                 });

  auto ret{util::make_unique<MLx>()};
  ret->setId(detector.id() + settings::kProcessorIdSep + util::createUUID());

  // XXX(damb): do not provide a sensor location (currently not required)
  ret->setEnvironment(detection.origin, nullptr, picks);

  ret->computeTimeWindow();
  ret->setGapInterpolation(detector.gapInterpolation());
  ret->setGapThreshold(detector.gapThreshold());
  ret->setGapTolerance(detector.gapTolerance());

  std::vector<std::string> sensorLocationStreamIdTokens;
  util::tokenizeWaveformStreamId(detection.sensorLocationStreamId,
                                 sensorLocationStreamIdTokens);
  const auto &sensorLocationBindings{Factory::loadSensorLocationConfig(
      bindings, sensorLocationStreamIdTokens[0],
      sensorLocationStreamIdTokens[1], sensorLocationStreamIdTokens[2],
      sensorLocationStreamIdTokens[3])};

  const auto &amplitudeProcessingConfig{
      sensorLocationBindings.amplitudeProcessingConfig};
  ret->setSaturationThreshold(
      amplitudeProcessingConfig.mlx.saturationThreshold);

  logging::TaggedMessage msg{detection.sensorLocationStreamId};
  // configure filter
  if (!amplitudeProcessingConfig.mlx.filter.value_or("").empty()) {
    auto filter{amplitudeProcessingConfig.mlx.filter.value()};
    util::replaceEscapedXMLFilterIdChars(filter);
    try {
      ret->setFilter(processing::createFilter(filter),
                     amplitudeProcessingConfig.mlx.initTime);
    } catch (processing::WaveformProcessor::BaseException &e) {
      msg.setText(e.what());
      throw BaseException{logging::to_string(msg)};
    }
    msg.setText(
        "Configured amplitude processor filter: filter=\"" + filter +
        "init_time=" + std::to_string(amplitudeProcessingConfig.mlx.initTime));
    SCDETECT_LOG_DEBUG_TAGGED(ret->id(), "%s", logging::to_string(msg).c_str());
  } else {
    msg.setText("Configured amplitude processor without filter: filter=\"\"");
    SCDETECT_LOG_DEBUG_TAGGED(ret->id(), "%s", logging::to_string(msg).c_str());
  }

  // choose arbitrarily the earliest pick w.r.t. a sensor location
  std::sort(std::begin(picks), std::end(picks),
            [](const decltype(picks)::value_type &lhs,
               const decltype(picks)::value_type &rhs) {
              return lhs->time().value() < rhs->time().value();
            });
  auto &earliestPick{picks.front()};
  try {
    const util::HorizontalComponents horizontalComponents{
        Client::Inventory::Instance(),   sensorLocationStreamIdTokens[0],
        sensorLocationStreamIdTokens[1], sensorLocationStreamIdTokens[2],
        sensorLocationStreamIdTokens[3], earliestPick->time().value()};
    for (const auto &s : horizontalComponents) {
      Processing::Stream stream;
      stream.init(s);

      AmplitudeProcessor::DeconvolutionConfig deconvolutionConfig;
      try {
        deconvolutionConfig =
            static_cast<AmplitudeProcessor::DeconvolutionConfig>(
                sensorLocationBindings.at(s->code()).deconvolutionConfig);
      } catch (std::out_of_range &e) {
        binding::StreamConfig::DeconvolutionConfig fallback;
        msg.setText(
            "failed to look up deconvolution configuration related bindings "
            "(channel code: \"" +
            s->code() + "\") required for amplitude processor configuration (" +
            e.what() + "); using fallback configuration, instead: \"" +
            fallback.debugString() + "\"");
        SCDETECT_LOG_WARNING_TAGGED(ret->id(), "%s",
                                    logging::to_string(msg).c_str());
        deconvolutionConfig =
            static_cast<AmplitudeProcessor::DeconvolutionConfig>(fallback);
      }

      ret->add(horizontalComponents.netCode(), horizontalComponents.staCode(),
               horizontalComponents.locCode(), stream, deconvolutionConfig);
    }
  } catch (Exception &e) {
    msg.setText(std::string{e.what()} +
                " (pick_time=" + earliestPick->time().value().iso() + ")");
    throw BaseException{logging::to_string(msg)};
  }

  return ret;
}

std::unique_ptr<AmplitudeProcessor> Factory::createRatioAmplitude(
    const binding::Bindings &bindings, const factory::Detection &detection,
    const detector::DetectorWaveformProcessor &detector) {
  assert(detection.origin);
  assert((detection.pickMap.size() == 1));

  std::vector<std::string> sensorLocationStreamIdTokens;
  util::tokenizeWaveformStreamId(detection.sensorLocationStreamId,
                                 sensorLocationStreamIdTokens);
  const auto &sensorLocationBindings{Factory::loadSensorLocationConfig(
      bindings, sensorLocationStreamIdTokens[0],
      sensorLocationStreamIdTokens[1], sensorLocationStreamIdTokens[2],
      sensorLocationStreamIdTokens[3])};
  const auto &amplitudeProcessingConfig{
      sensorLocationBindings.amplitudeProcessingConfig};

  logging::TaggedMessage msg{detection.sensorLocationStreamId};

  auto ret{util::make_unique<RatioAmplitude>()};

  const auto &pickPair{*std::begin(detection.pickMap)};
  const auto &processorId{pickPair.first};
  const auto &pick{pickPair.second};

  const auto *templateWaveformProcessor{detector.processor(processorId)};
  assert(templateWaveformProcessor);
  auto templateWaveform{templateWaveformProcessor->templateWaveform()};

  auto filter{amplitudeProcessingConfig.mrelative.filter};
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
    } else {
      msg.setText("Configured amplitude processor without filter: filter=\"\"");
      SCDETECT_LOG_DEBUG_TAGGED(ret->id(), "%s",
                                logging::to_string(msg).c_str());
    }
  } else {
    // use the filter from detector waveform processing as a fallback
    auto processingConfig{templateWaveform.processingConfig()};

    std::unique_ptr<waveform::DoubleFilter> cloned{
        templateWaveformProcessor->filter()->clone()};
    processingConfig.filter = std::move(cloned);
    processingConfig.initTime = templateWaveformProcessor->initTime();
    msg.setText(
        "Configured amplitude processor with detection processing filter: "
        "processor_id=" +
        templateWaveformProcessor->id());
    SCDETECT_LOG_DEBUG_TAGGED(ret->id(), "%s", logging::to_string(msg).c_str());
  }

  ret->setTemplateWaveform(templateWaveform);

  // XXX(damb): do not provide a sensor location (currently not required)
  ret->setEnvironment(detection.origin, nullptr, {pick.pick});

  ret->computeTimeWindow();
  ret->setGapInterpolation(detector.gapInterpolation());
  ret->setGapThreshold(detector.gapThreshold());
  ret->setGapTolerance(detector.gapTolerance());

  ret->setSaturationThreshold(
      amplitudeProcessingConfig.mrelative.saturationThreshold);

  return ret;
}

const binding::SensorLocationConfig &Factory::loadSensorLocationConfig(
    const binding::Bindings &bindings, const std::string &netCode,
    const std::string &staCode, const std::string &locCode,
    const std::string &chaCode) {
  try {
    return bindings.at(netCode, staCode, locCode, chaCode);
  } catch (std::out_of_range &e) {
    logging::TaggedMessage msg{
        util::join(netCode, staCode, locCode, chaCode),
        "failed to load bindings configuration: " + std::string{e.what()}};
    throw BaseException{logging::to_string(msg)};
  }
}

}  // namespace amplitude
}  // namespace detect
}  // namespace Seiscomp
