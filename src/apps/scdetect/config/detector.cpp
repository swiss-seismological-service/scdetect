#include "detector.h"

#include "../exception.h"
#include "../log.h"
#include "../util/waveform_stream_id.h"
#include "exception.h"
#include "validators.h"

namespace Seiscomp {
namespace detect {
namespace config {

StreamConfig::StreamConfig() {}

StreamConfig::StreamConfig(const std::string &wfStreamId,
                           const std::string &filter, const double initTime,
                           const TemplateStreamConfig &templateConfig,
                           const std::string &templateId)
    : wfStreamId{wfStreamId},
      initTime{initTime},
      filter{filter},
      templateConfig{templateConfig} {}

StreamConfig::StreamConfig(const boost::property_tree::ptree &pt,
                           const StreamConfig &defaults)
    : templateId{pt.get<std::string>("templateId", util::createUUID())

      },
      wfStreamId{pt.get<std::string>("waveformId")},
      initTime{pt.get<double>("initTime", defaults.initTime)},
      filter{pt.get_optional<std::string>("filter")},
      mergingThreshold{pt.get_optional<double>("mergingThreshold")},
      targetSamplingFrequency{
          pt.get_optional<double>("targetSamplingFrequency")} {
  templateConfig.phase =
      pt.get<std::string>("templatePhase", defaults.templateConfig.phase);
  templateConfig.wfStart =
      pt.get<double>("templateWaveformStart", defaults.templateConfig.wfStart);
  templateConfig.wfEnd =
      pt.get<double>("templateWaveformEnd", defaults.templateConfig.wfEnd);
  templateConfig.wfStreamId =
      pt.get<std::string>("templateWaveformId", wfStreamId);

  if (!targetSamplingFrequency && defaults.targetSamplingFrequency) {
    targetSamplingFrequency = defaults.targetSamplingFrequency;
  }

  if (!mergingThreshold && defaults.mergingThreshold) {
    mergingThreshold = defaults.mergingThreshold;
  }

  if (!filter && defaults.filter) {
    filter = defaults.filter;
  }

  templateConfig.filter = pt.get_optional<std::string>("templateFilter");
  if (!templateConfig.filter) {
    if (defaults.templateConfig.filter) {
      templateConfig.filter = defaults.templateConfig.filter;
    } else if (filter) {
      templateConfig.filter = filter;
    }
  }
}

bool StreamConfig::isValid() const {
  bool retval{true};
  try {
    util::WaveformStreamID{wfStreamId};
  } catch (detect::ValueException &e) {
    return false;
  }
  try {
    util::WaveformStreamID{templateConfig.wfStreamId};
  } catch (detect::ValueException &e) {
    return false;
  }

  if (mergingThreshold) {
    retval = validateXCorrThreshold(*mergingThreshold);
  }

  const auto validateFilter = [](const std::string &filterId) {
    if (filterId.empty()) {
      return true;
    }
    std::string err;
    return config::validateFilter(filterId, err);
  };

  if (filter) {
    retval = validateFilter(*filter);
  }

  if (templateConfig.filter) {
    retval = validateFilter(*templateConfig.filter);
  }

  return (retval && templateConfig.wfStart < templateConfig.wfEnd &&
          !templateConfig.phase.empty() && util::isGeZero(initTime));
}

bool DetectorConfig::isValid(size_t numStreamConfigs) const {
  return (
      validateXCorrThreshold(triggerOn) && validateXCorrThreshold(triggerOff) &&
      (!gapInterpolation ||
       (gapInterpolation && util::isGeZero(gapThreshold) &&
        util::isGeZero(gapTolerance) && gapThreshold < gapTolerance)) &&
      validateArrivalOffsetThreshold(arrivalOffsetThreshold) &&
      validateMinArrivals(minArrivals, static_cast<int>(numStreamConfigs)) &&
      validateLinkerMergingStrategy(mergingStrategy));
}

TemplateConfig::TemplateConfig(const boost::property_tree::ptree &pt,
                               const DetectorConfig &detectorDefaults,
                               const StreamConfig &streamDefaults,
                               const PublishConfig &publishDefaults)
    : _detectorId{pt.get<std::string>("detectorId", util::createUUID())},
      _originId(pt.get<std::string>("originId")) {
  _publishConfig.createArrivals =
      pt.get<bool>("createArrivals", publishDefaults.createArrivals);
  _publishConfig.createTemplateArrivals = pt.get<bool>(
      "createTemplateArrivals", publishDefaults.createTemplateArrivals);
  _publishConfig.originMethodId =
      pt.get<std::string>("methodId", publishDefaults.originMethodId);
  _publishConfig.createAmplitudes =
      pt.get<bool>("createAmplitudes", publishDefaults.createAmplitudes);
  _publishConfig.createMagnitudes =
      pt.get<bool>("createMagnitudes", publishDefaults.createMagnitudes &&
                                           publishDefaults.createAmplitudes);

  _detectorConfig.triggerOn =
      pt.get<double>("triggerOnThreshold", detectorDefaults.triggerOn);
  _detectorConfig.triggerOff =
      pt.get<double>("triggerOffThreshold", detectorDefaults.triggerOff);
  _detectorConfig.triggerDuration =
      pt.get<double>("triggerDuration", detectorDefaults.triggerDuration);
  _detectorConfig.timeCorrection =
      pt.get<double>("timeCorrection", detectorDefaults.timeCorrection);
  _detectorConfig.gapInterpolation =
      pt.get<bool>("gapInterpolation", detectorDefaults.gapInterpolation);
  _detectorConfig.gapThreshold =
      pt.get<double>("gapThreshold", detectorDefaults.gapThreshold);
  _detectorConfig.gapTolerance =
      pt.get<double>("gapTolerance", detectorDefaults.gapTolerance);
  _detectorConfig.maximumLatency =
      pt.get<double>("maximumLatency", detectorDefaults.maximumLatency);
  _detectorConfig.arrivalOffsetThreshold = pt.get<double>(
      "arrivalOffsetThreshold", detectorDefaults.arrivalOffsetThreshold);
  _detectorConfig.minArrivals =
      pt.get<int>("minimumArrivals", detectorDefaults.minArrivals);
  _detectorConfig.mergingStrategy =
      pt.get<std::string>("mergingStrategy", detectorDefaults.mergingStrategy);

  // patch stream defaults with detector config globals
  auto patchedStreamDefaults{streamDefaults};
  patchedStreamDefaults.initTime =
      pt.get<double>("initTime", streamDefaults.initTime);
  auto filter{pt.get_optional<std::string>("filter")};
  if (filter) {
    patchedStreamDefaults.filter = filter;
  }
  patchedStreamDefaults.targetSamplingFrequency =
      pt.get_optional<double>("targetSamplingFrequency");
  patchedStreamDefaults.mergingThreshold =
      pt.get_optional<double>("mergingThreshold");
  patchedStreamDefaults.templateConfig.phase =
      pt.get<std::string>("templatePhase", streamDefaults.templateConfig.phase);
  patchedStreamDefaults.templateConfig.wfStart = pt.get<double>(
      "templateWaveformStart", streamDefaults.templateConfig.wfStart);
  patchedStreamDefaults.templateConfig.wfEnd = pt.get<double>(
      "templateWaveformEnd", streamDefaults.templateConfig.wfEnd);
  auto templateFilter{pt.get_optional<std::string>("templateFilter")};
  if (templateFilter) {
    patchedStreamDefaults.templateConfig.filter = templateFilter;
  } else if (filter) {
    patchedStreamDefaults.templateConfig.filter = filter;
  }

  // initialize stream configs
  for (const auto &streamConfigPair : pt.get_child("streams")) {
    const auto &sc{streamConfigPair.second};

    std::string wfStreamId;
    try {
      StreamConfig streamConfig{sc, patchedStreamDefaults};
      _streamConfigs.emplace(streamConfig.wfStreamId, streamConfig);
      wfStreamId = streamConfig.wfStreamId;
    } catch (boost::property_tree::ptree_error &e) {
      throw config::ParserException{
          std::string{"Exception while parsing stream config: "} + e.what()};
    }

    if (!_streamConfigs[wfStreamId].isValid()) {
      throw config::ParserException{
          std::string{"Exception while parsing streamConfig: Invalid "
                      "stream configuration for stream: "} +
          wfStreamId};
    }
  }

  const auto maxArrivals{_streamConfigs.size()};
  if (_detectorConfig.minArrivals > static_cast<int>(maxArrivals)) {
    SCDETECT_LOG_WARNING_TAGGED(
        _detectorId,
        "Configured number of minimum arrivals exceeds number of configured "
        "streams (%d > %d). Resetting.",
        _detectorConfig.minArrivals, maxArrivals);
    _detectorConfig.minArrivals = maxArrivals;
  }

  if (!_detectorConfig.isValid(maxArrivals)) {
    throw config::ParserException{
        "Invalid template specific detector configuration"};
  }
}

std::string TemplateConfig::detectorId() const { return _detectorId; }

std::string TemplateConfig::originId() const { return _originId; }

DetectorConfig TemplateConfig::detectorConfig() const {
  return _detectorConfig;
}

PublishConfig TemplateConfig::publishConfig() const { return _publishConfig; }

TemplateConfig::reference TemplateConfig::at(const std::string &stream_id) {
  return _streamConfigs.at(stream_id);
}

TemplateConfig::const_reference TemplateConfig::at(
    const std::string &stream_id) const {
  return _streamConfigs.at(stream_id);
}

}  // namespace config
}  // namespace detect
}  // namespace Seiscomp
