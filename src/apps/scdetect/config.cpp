#include "config.h"

#include <boost/property_tree/exceptions.hpp>
#include <stdexcept>

#include "exception.h"
#include "log.h"
#include "utils.h"
#include "validators.h"

namespace Seiscomp {
namespace detect {

namespace config {

BaseException::BaseException() : Exception("base config exception") {}

ParserException::ParserException()
    : BaseException{"error while parsing configuration"} {}

}  // namespace config

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
    : templateId{pt.get<std::string>("templateId", utils::createUUID())

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
  if (!templateConfig.filter && defaults.templateConfig.filter) {
    templateConfig.filter = defaults.templateConfig.filter;
  }
}

bool StreamConfig::isValid() const {
  bool retval{true};
  try {
    retval = utils::WaveformStreamID{wfStreamId}.isValid();
  } catch (ValueException &e) {
    return false;
  }
  try {
    retval = utils::WaveformStreamID{templateConfig.wfStreamId}.isValid();
  } catch (ValueException &e) {
    return false;
  }

  if (mergingThreshold) {
    retval = config::validateXCorrThreshold(*mergingThreshold);
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
          !templateConfig.phase.empty() && utils::isGeZero(initTime));
}

bool DetectorConfig::isValid(size_t numStreamConfigs) const {
  return (config::validateXCorrThreshold(triggerOn) &&
          config::validateXCorrThreshold(triggerOff) &&
          (!gapInterpolation ||
           (gapInterpolation && utils::isGeZero(gapThreshold) &&
            utils::isGeZero(gapTolerance) && gapThreshold < gapTolerance)) &&
          config::validateArrivalOffsetThreshold(arrivalOffsetThreshold) &&
          config::validateMinArrivals(minArrivals,
                                      static_cast<int>(numStreamConfigs)) &&
          config::validateLinkerMergingStrategy(mergingStrategy));
}

TemplateConfig::TemplateConfig(const boost::property_tree::ptree &pt,
                               const DetectorConfig &detectorDefaults,
                               const StreamConfig &streamDefaults,
                               const PublishConfig &publishDefaults)
    : _detectorId{pt.get<std::string>("detectorId", utils::createUUID())},
      _originId(pt.get<std::string>("originId")) {
  _publishConfig.createArrivals =
      pt.get<bool>("createArrivals", publishDefaults.createArrivals);
  _publishConfig.createTemplateArrivals = pt.get<bool>(
      "createTemplateArrivals", publishDefaults.createTemplateArrivals);
  _publishConfig.originMethodId =
      pt.get<std::string>("methodId", publishDefaults.originMethodId);
  _publishConfig.createAmplitudes =
      pt.get<bool>("calculateAmplitudes", publishDefaults.createAmplitudes);

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
  patchedStreamDefaults.filter = pt.get_optional<std::string>("filter");
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
  patchedStreamDefaults.templateConfig.filter =
      pt.get_optional<std::string>("templateFilter");

  // initialize stream configs
  for (const auto &streamConfigPair : pt.find("streams")->second) {
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

/* ------------------------------------------------------------------------- */
TemplateFamilyConfig::ReferenceConfig::ReferenceConfig(
    const boost::property_tree::ptree &pt,
    const TemplateConfigs &templateConfigs,
    const ReferenceConfig::StreamConfig &streamDefaults) {
  const auto detectorId{pt.get_optional<std::string>("detectorId")};
  const auto origId{pt.get_optional<std::string>("originId")};
  if (detectorId && origId) {
    throw config::ParserException{
        "invalid configuration: both \"detectorId\" and \"originId\" "
        "specified"};
  }

  if (origId && !origId.value().empty()) {
    // explicit configuration
    for (const auto &streamConfigPt : pt.get_child("streams")) {
      const auto &pt{streamConfigPt.second};
      StreamConfig streamConfig;
      try {
        const auto waveformId{
            utils::WaveformStreamID{pt.get<std::string>("templateWaveformId")}};
        streamConfig.waveformId = waveformId.sensorLocationStreamId();
      } catch (ValueException &e) {
        throw config::ParserException{"invalid configuration: " +
                                      std::string{e.what()}};
      }

      streamConfig.phase =
          pt.get<std::string>("templatePhase", streamDefaults.phase);
      if (streamConfig.phase.empty()) {
        streamConfig.phase = streamDefaults.phase;
      }
      streamConfig.waveformStart =
          pt.get<double>("templateWaveformStart", streamDefaults.waveformStart);
      streamConfig.waveformEnd =
          pt.get<double>("templateWaveformEnd", streamDefaults.waveformEnd);

      streamConfigs.emplace(streamConfig);
    }

    originId = *origId;
  } else if (detectorId && !detectorId.value().empty()) {
    // indirect configuration referencing a detector config
    const auto it{templateConfigs.find(*detectorId)};
    if (it == std::end(templateConfigs)) {
      throw config::ParserException{
          "invalid configuration: invalid \"detectorId\": " + *detectorId};
    }

    for (const auto &streamConfigPt : pt.get_child("streams")) {
      const auto &pt{streamConfigPt.second};

      StreamConfig streamConfig;
      detect::StreamConfig detectorStreamConfig;
      try {
        const auto waveformId{
            utils::WaveformStreamID{pt.get_value<std::string>()}};

        detectorStreamConfig = it->second.at(utils::to_string(waveformId));

        streamConfig.waveformId = waveformId.sensorLocationStreamId();

      } catch (std::out_of_range &e) {
        throw config::ParserException{
            "invalid configuration: failed to look up stream configuration for "
            "stream: " +
            pt.get_value<std::string>()};
      } catch (ValueException &e) {
        throw config::ParserException{"invalid configuration: " +
                                      std::string{e.what()}};
      }

      streamConfig.phase = detectorStreamConfig.templateConfig.phase;
      streamConfig.waveformStart = detectorStreamConfig.templateConfig.wfStart;
      streamConfig.waveformEnd = detectorStreamConfig.templateConfig.wfEnd;
    }

    if (streamConfigs.empty()) {
      throw config::ParserException{
          "invalid configuration: no stream configuration found"};
    }

    originId = it->second.originId();
  } else {
    throw config::ParserException{
        "invalid configuration: neither \"detectorId\" nor \"originId\" "
        "specified"};
  }
}

TemplateFamilyConfig::TemplateFamilyConfig(
    const boost::property_tree::ptree &pt,
    const TemplateConfigs &templateConfigs,
    const ReferenceConfig::StreamConfig &streamDefaults)
    : _id{pt.get<std::string>("id", utils::createUUID())} {
  loadReferenceConfigs(pt.get_child("references"), templateConfigs,
                       streamDefaults);
}

const std::string &TemplateFamilyConfig::id() const { return _id; }

void TemplateFamilyConfig::loadReferenceConfigs(
    const boost::property_tree::ptree &pt,
    const TemplateConfigs &templateConfigs,
    const ReferenceConfig::StreamConfig &streamDefaults) {
  for (const auto &referenceConfigPt : pt) {
    const auto &pt{referenceConfigPt.second};

    _referenceConfigs.emplace(
        ReferenceConfig{pt, templateConfigs, streamDefaults});
  }
}

}  // namespace detect
}  // namespace Seiscomp
