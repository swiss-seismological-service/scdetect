#include "config.h"

#include <boost/property_tree/exceptions.hpp>
#include <stdexcept>
#include <vector>

#include "exception.h"
#include "log.h"
#include "util/util.h"
#include "util/waveform_stream_id.h"
#include "validators.h"

namespace Seiscomp {
namespace detect {

namespace config {

BaseException::BaseException() : Exception("base config exception") {}

ParserException::ParserException()
    : BaseException{"error while parsing configuration"} {}

ValidationError::ValidationError() : BaseException{"validation error"} {}

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
  if (!templateConfig.filter && defaults.templateConfig.filter) {
    templateConfig.filter = defaults.templateConfig.filter;
  }
}

bool StreamConfig::isValid() const {
  bool retval{true};
  try {
    retval = util::WaveformStreamID{wfStreamId}.isValid();
  } catch (ValueException &e) {
    return false;
  }
  try {
    retval = util::WaveformStreamID{templateConfig.wfStreamId}.isValid();
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
          !templateConfig.phase.empty() && util::isGeZero(initTime));
}

bool DetectorConfig::isValid(size_t numStreamConfigs) const {
  return (config::validateXCorrThreshold(triggerOn) &&
          config::validateXCorrThreshold(triggerOff) &&
          (!gapInterpolation ||
           (gapInterpolation && util::isGeZero(gapThreshold) &&
            util::isGeZero(gapTolerance) && gapThreshold < gapTolerance)) &&
          config::validateArrivalOffsetThreshold(arrivalOffsetThreshold) &&
          config::validateMinArrivals(minArrivals,
                                      static_cast<int>(numStreamConfigs)) &&
          config::validateLinkerMergingStrategy(mergingStrategy));
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

/* ------------------------------------------------------------------------- */
TemplateFamilyConfig::ReferenceConfig::TemplateConfigsIdx
    TemplateFamilyConfig::ReferenceConfig::_templateConfigsIdx;

const TemplateFamilyConfig::AllowedMagnitudeTypes
    TemplateFamilyConfig::_allowedMagnitudeTypes{"MLx"};

TemplateFamilyConfig::ReferenceConfig::ReferenceConfig(
    const boost::property_tree::ptree &pt,
    const std::vector<TemplateConfig> &templateConfigs,
    const ReferenceConfig::SensorLocationConfig &sensorLocationDefaults) {
  // detector identifier
  const auto dId{pt.get_optional<std::string>("detectorId")};
  // origin identifier
  const auto oId{pt.get_optional<std::string>("originId")};
  if (dId && oId) {
    throw config::ParserException{
        "invalid configuration: both \"detectorId\" and \"originId\" "
        "specified"};
  }

  if (oId && !oId.value().empty()) {
    // explicit configuration
    for (const auto &streamConfigPair : pt.get_child("streams")) {
      const auto &streamConfigPt{streamConfigPair.second};
      SensorLocationConfig sensorLocationConfig;
      try {
        const auto waveformId{util::WaveformStreamID{
            streamConfigPt.get<std::string>("templateWaveformId")}};
        sensorLocationConfig.waveformId = waveformId.sensorLocationStreamId();
        sensorLocationConfig.channelId = waveformId.chaCode();
      } catch (ValueException &e) {
        throw config::ValidationError{"invalid configuration: " +
                                      std::string{e.what()}};
      }

      sensorLocationConfig.phase = streamConfigPt.get<std::string>(
          "templatePhase", sensorLocationDefaults.phase);
      if (sensorLocationConfig.phase.empty()) {
        sensorLocationConfig.phase = sensorLocationDefaults.phase;
      }
      sensorLocationConfig.waveformStart = streamConfigPt.get<double>(
          "templateWaveformStart", sensorLocationDefaults.waveformStart);
      sensorLocationConfig.waveformEnd = streamConfigPt.get<double>(
          "templateWaveformEnd", sensorLocationDefaults.waveformEnd);

      // XXX(damb): explicit references (i.e. those not referencing a detector
      // configuration) do not take limits into account.

      sensorLocationConfigs.emplace(sensorLocationConfig);
    }

    originId = *oId;
  } else if (dId && !dId.value().empty()) {
    if (!indexed()) {
      createIndex(templateConfigs);
    }

    // indirect configuration referencing a detector config
    const auto it{_templateConfigsIdx.find(*dId)};
    if (it == std::end(_templateConfigsIdx)) {
      throw config::ParserException{
          "invalid configuration: invalid \"detectorId\": " + *detectorId};
    }

    for (const auto &streamConfigPair : pt.get_child("streams")) {
      const auto &streamConfigPt{streamConfigPair.second};

      SensorLocationConfig sensorLocationConfig;
      detect::StreamConfig detectorStreamConfig;
      try {
        const auto waveformId{util::WaveformStreamID{
            streamConfigPt.get<std::string>("templateWaveformId")}};

        detectorStreamConfig = it->second->at(util::to_string(waveformId));

        sensorLocationConfig.waveformId = waveformId.sensorLocationStreamId();
        sensorLocationConfig.channelId = waveformId.chaCode();
        sensorLocationConfig.lowerLimit =
            streamConfigPt.get_optional<double>("lowerLimit");
        if (!sensorLocationConfig.lowerLimit) {
          sensorLocationConfig.lowerLimit = sensorLocationDefaults.lowerLimit;
        }
        sensorLocationConfig.upperLimit =
            streamConfigPt.get_optional<double>("upperLimit");
        if (!sensorLocationConfig.upperLimit) {
          sensorLocationConfig.upperLimit = sensorLocationDefaults.upperLimit;
        }
        if (sensorLocationConfig.lowerLimit &&
            sensorLocationConfig.upperLimit &&
            (*sensorLocationConfig.upperLimit <=
             *sensorLocationConfig.lowerLimit)) {
          throw config::ValidationError{
              "invalid configuration: \"upperLimit\" must be greater than "
              "\"lowerLimit\""};
        }

      } catch (std::out_of_range &e) {
        throw config::ParserException{
            "invalid configuration: failed to look up stream configuration for "
            "stream: " +
            streamConfigPt.get<std::string>("templateWaveformId")};
      } catch (ValueException &e) {
        throw config::ValidationError{"invalid configuration: " +
                                      std::string{e.what()}};
      }

      sensorLocationConfig.phase = detectorStreamConfig.templateConfig.phase;
      sensorLocationConfig.waveformStart =
          detectorStreamConfig.templateConfig.wfStart;
      sensorLocationConfig.waveformEnd =
          detectorStreamConfig.templateConfig.wfEnd;

      sensorLocationConfigs.emplace(sensorLocationConfig);
    }

    if (sensorLocationConfigs.empty()) {
      throw config::ParserException{
          "invalid configuration: no stream configuration found"};
    }

    originId = it->second->originId();
    detectorId = *dId;
  } else {
    throw config::ParserException{
        "invalid configuration: neither \"detectorId\" nor \"originId\" "
        "specified"};
  }
}

bool TemplateFamilyConfig::ReferenceConfig::referencesDetector() const {
  return static_cast<bool>(detectorId);
}

void TemplateFamilyConfig::ReferenceConfig::createIndex(
    const TemplateFamilyConfig::ReferenceConfig::TemplateConfigs
        &templateConfigs) {
  for (auto it{std::begin(templateConfigs)}; it != std::end(templateConfigs);
       ++it) {
    _templateConfigsIdx.emplace(it->detectorId(), it);
  }
}

bool TemplateFamilyConfig::ReferenceConfig::indexed() const {
  return !_templateConfigsIdx.empty();
}

TemplateFamilyConfig::TemplateFamilyConfig(
    const boost::property_tree::ptree &pt,
    const std::vector<TemplateConfig> &templateConfigs,
    const ReferenceConfig::SensorLocationConfig &sensorLocationDefaults)
    : _id{pt.get<std::string>("id", util::createUUID())},
      _magnitudeType{pt.get<std::string>("magnitudeType", "MLx")} {
  validateMagnitudeType(_magnitudeType);

  // parse template family configuration defaults
  const auto lowerLimitDefault{pt.get_optional<double>("lowerLimit")};
  const auto upperLimitDefault{pt.get_optional<double>("upperLimit")};
  if (lowerLimitDefault && upperLimitDefault &&
      *upperLimitDefault <= *lowerLimitDefault) {
    throw config::ValidationError{
        "invalid configuration: `\"lowerLimit\" must be greater than "
        "\"upperLimit\""};
  }
  ReferenceConfig::SensorLocationConfig sensorLocationDefaultsWithGlobals{
      sensorLocationDefaults};
  sensorLocationDefaultsWithGlobals.lowerLimit = lowerLimitDefault;
  sensorLocationDefaultsWithGlobals.upperLimit = upperLimitDefault;

  loadReferenceConfigs(pt.get_child("references"), templateConfigs,
                       sensorLocationDefaultsWithGlobals);
}

const std::string &TemplateFamilyConfig::id() const { return _id; }

const std::string &TemplateFamilyConfig::magnitudeType() const {
  return _magnitudeType;
}

void TemplateFamilyConfig::loadReferenceConfigs(
    const boost::property_tree::ptree &pt,
    const std::vector<TemplateConfig> &templateConfigs,
    const ReferenceConfig::SensorLocationConfig &sensorLocationDefaults) {
  bool hasDetectorReference{false};
  for (const auto &referenceConfigPair : pt) {
    const auto &referenceConfigPt{referenceConfigPair.second};

    ReferenceConfig referenceConfig{referenceConfigPt, templateConfigs,
                                    sensorLocationDefaults};
    if (referenceConfig.referencesDetector()) {
      hasDetectorReference = true;
    }
    _referenceConfigs.emplace(referenceConfig);
  }

  if (!hasDetectorReference) {
    throw config::ValidationError{
        "missing detector reference: template families must at least define a "
        "single reference to a detector"};
  }
}

void TemplateFamilyConfig::validateMagnitudeType(
    const std::string &magnitudeType) {
  if (_allowedMagnitudeTypes.find(magnitudeType) ==
      std::end(_allowedMagnitudeTypes)) {
    throw config::ValidationError{
        "invalid configuration: invalid magnitude type: " + magnitudeType};
  }
}

}  // namespace detect
}  // namespace Seiscomp
