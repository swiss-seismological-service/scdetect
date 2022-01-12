#include "template_family.h"

#include <seiscomp/client/inventory.h>
#include <seiscomp/core/exceptions.h>
#include <seiscomp/core/genericrecord.h>
#include <seiscomp/core/timewindow.h>
#include <seiscomp/datamodel/origin.h>
#include <seiscomp/datamodel/waveformstreamid.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "amplitude/factory.h"
#include "binding.h"
#include "builder.h"
#include "eventstore.h"
#include "log.h"
#include "settings.h"
#include "util/horizontal_components.h"
#include "util/util.h"
#include "util/waveform_stream_id.h"

namespace Seiscomp {
namespace detect {

const TemplateFamily::Builder::MagnitudeTypeMap
    TemplateFamily::Builder::_magnitudeTypeMap{{"MLx", {"MLhc", "MLh"}}};

TemplateFamily::Builder::Builder(
    config::TemplateFamilyConfig templateFamilyConfig)
    : _templateFamilyConfig{std::move(templateFamilyConfig)} {
  // XXX(damb): Using `new` to access a non-public ctor; see also
  // https://abseil.io/tips/134
  setProduct(std::unique_ptr<TemplateFamily>{new TemplateFamily{}});
}

TemplateFamily::Builder& TemplateFamily::Builder::setId(
    const boost::optional<std::string>& id) {
  product()->_id = id.value_or(_templateFamilyConfig.id());
  return *this;
}

TemplateFamily::Builder& TemplateFamily::Builder::setLimits(
    const boost::optional<double>& lower,
    const boost::optional<double>& upper) {
  for (const auto& referenceConfig : _templateFamilyConfig) {
    const auto origin{EventStore::Instance().getWithChildren<DataModel::Origin>(
        referenceConfig.originId)};
    if (!origin) {
      throw builder::BaseException{"failed to find origin with id: " +
                                   referenceConfig.originId};
    }

    for (const auto& sensorLocationConfig :
         referenceConfig.sensorLocationConfigs) {
      const auto& sensorLocationId{sensorLocationConfig.waveformId};

      auto& originConfig{_members[origin->publicID()]};
      auto& member{originConfig[sensorLocationId]};
      member.config.lowerLimit = lower;
      if (!member.config.lowerLimit) {
        member.config.lowerLimit = sensorLocationConfig.lowerLimit;
      }
      member.config.upperLimit = upper;
      if (!member.config.upperLimit) {
        member.config.upperLimit = sensorLocationConfig.upperLimit;
      }
    }
  }

  return *this;
}

TemplateFamily::Builder& TemplateFamily::Builder::setStationMagnitudes(
    const boost::optional<std::string>& magnitudeType) {
  auto configuredMagnitudeType{
      magnitudeType.value_or(_templateFamilyConfig.magnitudeType())};
  auto matchingMagnitudeTypes{_magnitudeTypeMap.at(configuredMagnitudeType)};
  for (const auto& referenceConfig : _templateFamilyConfig) {
    const auto origin{EventStore::Instance().getWithChildren<DataModel::Origin>(
        referenceConfig.originId)};
    if (!origin) {
      throw builder::BaseException{"failed to find origin with id: " +
                                   referenceConfig.originId};
    }

    using AvailableMatchingMagnitudes =
        std::unordered_map<std::string, DataModel::StationMagnitudeCPtr>;
    AvailableMatchingMagnitudes availableMatchingMagnitudes;
    // collect available matching station magnitudes
    for (const auto& sensorLocationConfig :
         referenceConfig.sensorLocationConfigs) {
      const auto& sensorLocationId{sensorLocationConfig.waveformId};
      for (size_t i = 0; i < origin->stationMagnitudeCount(); ++i) {
        const auto* stationMagnitude{origin->stationMagnitude(i)};
        const auto amplitudeId{stationMagnitude->amplitudeID()};

        const auto amplitude{
            EventStore::Instance().get<DataModel::Amplitude>(amplitudeId)};
        if (!amplitude) {
          continue;
        }

        bool matchingMagnitudeType{std::find(std::begin(matchingMagnitudeTypes),
                                             std::end(matchingMagnitudeTypes),
                                             stationMagnitude->type()) !=
                                   matchingMagnitudeTypes.end()};
        if (!matchingMagnitudeType) {
          continue;
        }

        try {
          DataModel::WaveformStreamID waveformStreamId;
          try {
            waveformStreamId = amplitude->waveformID();
          } catch (Core::ValueException&) {
            try {
              waveformStreamId = stationMagnitude->waveformID();
            } catch (Core::ValueException&) {
              // missing waveform stream identifier
              continue;
            }
          }

          auto magnitudeSensorLocationId{util::getSensorLocationStreamId(
              util::WaveformStreamID{waveformStreamId})};
          if (magnitudeSensorLocationId != sensorLocationId) {
            continue;
          }

          availableMatchingMagnitudes.emplace(stationMagnitude->type(),
                                              stationMagnitude);

        } catch (std::out_of_range&) {
          continue;
        }
      }

      // assign station magnitudes
      for (const auto& mType : matchingMagnitudeTypes) {
        try {
          auto& magnitudeCandidate{availableMatchingMagnitudes.at(mType)};

          auto& originConfig{_members[origin->publicID()]};
          auto& member{originConfig[sensorLocationId]};
          member.magnitude = magnitudeCandidate;
          break;
        } catch (std::out_of_range&) {
          continue;
        }
      }
    }
  }

  product()->_magnitudeType = configuredMagnitudeType;

  return *this;
}

TemplateFamily::Builder& TemplateFamily::Builder::setAmplitudes(
    WaveformHandlerIface* waveformHandler, const binding::Bindings& bindings) {
  assert(waveformHandler);

  for (const auto& referenceConfig : _templateFamilyConfig) {
    const auto origin{EventStore::Instance().getWithChildren<DataModel::Origin>(
        referenceConfig.originId)};
    if (!origin) {
      throw builder::BaseException{"failed to find origin with id: " +
                                   referenceConfig.originId};
    }

    for (const auto& sensorLocationConfig :
         referenceConfig.sensorLocationConfigs) {
      // use phase code to determine arrival time (i.e. actually the
      // corresponding pick time is used)
      const auto& arrivalPhase{sensorLocationConfig.phase};

      logging::TaggedMessage msg{sensorLocationConfig.waveformId};

      DataModel::PickCPtr pick;
      DataModel::SensorLocationCPtr sensorLocation;

      std::vector<std::string> tokens;
      util::tokenizeWaveformStreamId(sensorLocationConfig.waveformId, tokens);
      if (tokens.size() != 3) {
        continue;
      }

      try {
        getPickAndSensorLocation(origin.get(), tokens[0], tokens[1], tokens[2],
                                 arrivalPhase, sensorLocation, pick);
      } catch (const builder::NoSensorLocation& e) {
        // wrap exception into message
        msg.setText(e.what());
        throw builder::NoSensorLocation{logging::to_string(msg)};
      }

      if (!pick) {
        msg.setText("failed to load pick from event parameters");
        throw builder::NoPick{logging::to_string(msg)};
      }

      const auto arrivalTime{pick->time().value()};

      amplitude::factory::Detection referenceDetection;

      referenceDetection.sensorLocationStreamId = util::join(
          tokens[0], tokens[1], tokens[2],
          util::getBandAndSourceCode(sensorLocationConfig.channelId));

      referenceDetection.origin = origin;
      referenceDetection.pickMap.emplace(
          "", amplitude::factory::Detection::Pick{"", pick});

      amplitude::factory::DetectorConfig detectorConfig;
      detectorConfig.gapInterpolation = false;
      Core::TimeWindow tw{
          arrivalTime + Core::TimeSpan{sensorLocationConfig.waveformStart},
          arrivalTime + Core::TimeSpan{sensorLocationConfig.waveformEnd}};
      auto proc{amplitude::factory::createMLx(bindings, referenceDetection,
                                              detectorConfig, tw)};

      proc->setResultCallback(
          [this](const AmplitudeProcessor* proc, const Record* rec,
                 AmplitudeProcessor::AmplitudeCPtr amplitude) {
            storeAmplitude(proc, rec, amplitude);
          });

      proc->setId(_templateFamilyConfig.id() + settings::kProcessorIdSep +
                  util::createUUID());

      proc->setEnvironment(origin, sensorLocation, {pick});

      try {
        util::HorizontalComponents horizontalComponents{
            Client::Inventory::Instance(),  tokens[0],  tokens[1], tokens[2],
            sensorLocationConfig.channelId, arrivalTime};

        WaveformHandlerIface::ProcessingConfig processingConfig;
        // let the amplitude processor decide how to process the waveform
        processingConfig.demean = false;
        // load waveforms for the horizontal components and feed the data to
        // the processor
        for (auto s : horizontalComponents) {
          auto record{waveformHandler->get(
              horizontalComponents.netCode(), horizontalComponents.staCode(),
              horizontalComponents.locCode(), s->code(),
              proc->safetyTimeWindow(), processingConfig)};
          proc->feed(record.get());
        }
      } catch (processing::WaveformProcessor::BaseException& e) {
        msg.setText("failed to load data");
        throw builder::BaseException{logging::to_string(msg)};
      } catch (Exception& e) {
        msg.setText("failed to load streams from inventory for time: " +
                    arrivalTime.iso());
        throw builder::NoStream{logging::to_string(msg)};
      }

      if (proc->status() != processing::WaveformProcessor::Status::kFinished) {
        msg.setText("failed to compute rms amplitude: status=" +
                    std::to_string(util::asInteger(proc->status())));
        throw builder::BaseException{logging::to_string(msg)};
      }
    }
  }
  return *this;
}

void TemplateFamily::Builder::finalize() {
  // establish reference to detector
  for (const auto& referenceConfig : _templateFamilyConfig) {
    if (referenceConfig.detectorId) {
      auto it{_members.find(referenceConfig.originId)};
      if (it == _members.end()) {
        continue;
      }

      for (auto& sensorLocationConfigPair : it->second) {
        sensorLocationConfigPair.second.config.detectorId =
            referenceConfig.detectorId;
      }
    }
  }

  for (auto& configPair : _members) {
    for (auto& sensorLocationConfigPair : configPair.second) {
      const auto& sensorLocationId{sensorLocationConfigPair.first};
      auto& member{sensorLocationConfigPair.second};
      if (member.amplitude && member.magnitude) {
        member.config.sensorLocationId = sensorLocationId;
        product()->_members.push_back(member);
      }
    }
  }
}

void TemplateFamily::Builder::getPickAndSensorLocation(
    const DataModel::Origin* origin, const std::string& netCode,
    const std::string& staCode, const std::string& locCode,
    const std::string& phase, DataModel::SensorLocationCPtr& sensorLocation,
    DataModel::PickCPtr& pick) {
  assert(static_cast<bool>(origin));
  for (size_t i = 0; i < origin->arrivalCount(); ++i) {
    const auto* arrival{origin->arrival(i)};
    if (arrival->phase() != phase) {
      continue;
    }
    auto p{EventStore::Instance().get<DataModel::Pick>(arrival->pickID())};
    if (!p) {
      continue;
    }
    // validate both arrival and pick
    try {
      if (p->evaluationMode() != DataModel::MANUAL &&
          (arrival->weight() == 0 || !arrival->timeUsed())) {
        continue;
      }
    } catch (Core::ValueException& e) {
      continue;
    }

    // validate the pick's sensor location
    auto* templateWaveformSensorLocation{
        Client::Inventory::Instance()->getSensorLocation(
            netCode, staCode, locCode, p->time().value())};
    if (!static_cast<bool>(templateWaveformSensorLocation)) {
      throw builder::NoSensorLocation{
          "failed to find sensor location in inventory for time: " +
          p->time().value().iso()};
    }

    auto pickWaveformId{p->waveformID()};
    auto* pickWaveformSensorLocation{
        Client::Inventory::Instance()->getSensorLocation(
            pickWaveformId.networkCode(), pickWaveformId.stationCode(),
            pickWaveformId.locationCode(), p->time().value())};
    if (!static_cast<bool>(pickWaveformSensorLocation) ||
        *templateWaveformSensorLocation != *pickWaveformSensorLocation) {
      continue;
    }

    pick = p;
    sensorLocation = templateWaveformSensorLocation;
    break;
  }
}

void TemplateFamily::Builder::storeAmplitude(
    const AmplitudeProcessor* processor, const Record* record,
    const AmplitudeProcessor::AmplitudeCPtr& amplitude) {
  Core::TimeWindow tw{
      amplitude->time.reference - Core::TimeSpan{amplitude->time.begin},
      amplitude->time.reference + Core::TimeSpan{amplitude->time.end}};
  SCDETECT_LOG_DEBUG_TAGGED(
      processor->id(),
      "Creating amplitude (value=%f, starttime=%s, endtime=%s) ...",
      amplitude->value.value, tw.startTime().iso().c_str(),
      tw.endTime().iso().c_str());

  DataModel::AmplitudePtr amp{DataModel::Amplitude::Create()};
  if (!amp) {
    SCDETECT_LOG_WARNING_PROCESSOR(
        processor, "Internal error: duplicate amplitude identifier");
    return;
  }

  amp->setType(processor->type());
  amp->setUnit(processor->unit());
  if (amplitude->waveformStreamIds) {
    if (amplitude->waveformStreamIds.value().size() == 1) {
      const util::WaveformStreamID waveformStreamId{
          amplitude->waveformStreamIds.value()[0]};
      amp->setWaveformID(DataModel::WaveformStreamID{
          waveformStreamId.netCode(), waveformStreamId.staCode(),
          waveformStreamId.locCode(), waveformStreamId.chaCode(), ""});
    }
  }

  amp->setAmplitude(DataModel::RealQuantity{
      amplitude->value.value, boost::none, amplitude->value.lowerUncertainty,
      amplitude->value.upperUncertainty, boost::none});
  amp->setTimeWindow(DataModel::TimeWindow{
      amplitude->time.reference, amplitude->time.begin, amplitude->time.end});

  amp->setSnr(amplitude->snr);
  if (amplitude->dominantPeriod) {
    amp->setPeriod(DataModel::RealQuantity{*amplitude->dominantPeriod});
  }
  processor->finalize(amp.get());

  auto& originConfig{_members[processor->environment().hypocenter->publicID()]};
  auto sensorLocationId{util::getSensorLocationStreamId(
      util::WaveformStreamID{record->networkCode(), record->stationCode(),
                             record->locationCode(), record->channelCode()})};
  auto& member{originConfig[sensorLocationId]};
  member.amplitude = amp;
}

/* ------------------------------------------------------------------------- */
bool TemplateFamily::Member::referencesDetector() const {
  return static_cast<bool>(config.detectorId);
}

TemplateFamily::TemplateFamily() {}

TemplateFamily::Builder TemplateFamily::Create(
    const config::TemplateFamilyConfig& templateFamilyConfig) {
  return Builder(templateFamilyConfig);
}

const std::string& TemplateFamily::id() const { return _id; }

const std::string& TemplateFamily::magnitudeType() const {
  return _magnitudeType;
}

}  // namespace detect
}  // namespace Seiscomp
