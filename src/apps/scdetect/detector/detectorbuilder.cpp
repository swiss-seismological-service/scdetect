#include "detectorbuilder.h"

#include <seiscomp/client/inventory.h>
#include <seiscomp/core/datetime.h>
#include <seiscomp/core/exceptions.h>
#include <seiscomp/core/timewindow.h>

#include <boost/none.hpp>
#include <boost/optional.hpp>
#include <cmath>
#include <memory>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>

#include "../eventstore.h"
#include "../log.h"
#include "../operator/ringbuffer.h"
#include "../settings.h"
#include "../utils.h"
#include "detectorwaveformprocessor.h"

namespace Seiscomp {
namespace detect {
namespace detector {

DetectorBuilder::DetectorBuilder(const std::string &id,
                                 const std::string &originId,
                                 const std::string &originMethodId)
    : _originId{originId} {
  DataModel::OriginCPtr origin{
      EventStore::Instance().getWithChildren<DataModel::Origin>(originId)};
  if (!origin) {
    SCDETECT_LOG_WARNING("Origin %s not found.", originId.c_str());
    throw builder::BaseException{std::string{"Error while assigning origin: "} +
                                 originId};
  }

  // XXX(damb): Using `new` to access a non-public ctor; see also
  // https://abseil.io/tips/134
  _product = std::unique_ptr<DetectorWaveformProcessor>(
      new DetectorWaveformProcessor{id, originMethodId, origin});
}

DetectorBuilder &DetectorBuilder::setConfig(const DetectorConfig &config,
                                            bool playback) {
  _product->_config = config;

  _product->_enabled = config.enabled;

  _product->_detector.setMergingStrategy(
      _mergingStrategyLookupTable.at(config.mergingStrategy));

  // configure playback related facilities
  if (playback) {
    _product->_detector.setMaxLatency(boost::none);
  } else {
    if (config.maximumLatency > 0) {
      _product->_detector.setMaxLatency(Core::TimeSpan{config.maximumLatency});
    }
  }

  return *this;
}

DetectorBuilder &DetectorBuilder::setEventParameters() {
  _product->_event =
      EventStore::Instance().getEvent(_product->_origin->publicID());
  if (!_product->_event) {
    auto msg{std::string{"No event associated with origin: "} + _originId};

    SCDETECT_LOG_WARNING("%s", msg.c_str());
    throw builder::BaseException{msg};
  }

  _product->_magnitude = EventStore::Instance().get<DataModel::Magnitude>(
      _product->_event->preferredMagnitudeID());
  if (!_product->_magnitude) {
    auto msg{std::string{"No magnitude associated with event: "} +
             _product->_event->publicID() + std::string{" (origin="} +
             _originId + std::string{")"}};

    SCDETECT_LOG_WARNING("%s", msg.c_str());
    throw builder::BaseException{msg};
  }

  return *this;
}

DetectorBuilder &DetectorBuilder::setStream(const std::string &streamId,
                                            const StreamConfig &streamConfig,
                                            WaveformHandlerIfacePtr &wfHandler,
                                            bool debugMode) {
  const auto &templateStreamId{streamConfig.templateConfig.wfStreamId};
  utils::WaveformStreamID templateWfStreamId{templateStreamId};

  // TODO(damb): Rather go for a logging adapter approach.
  std::string logPrefix{streamId + std::string{" ("} + templateStreamId +
                        std::string{"): "}};

  // configure pick from arrival
  DataModel::PickPtr pick;
  DataModel::WaveformStreamID pickWaveformId;
  DataModel::ArrivalPtr arrival;
  for (size_t i = 0; i < _product->_origin->arrivalCount(); ++i) {
    arrival = _product->_origin->arrival(i);

    if (arrival->phase().code() != streamConfig.templateConfig.phase) {
      continue;
    }

    pick = EventStore::Instance().get<DataModel::Pick>(arrival->pickID());
    if (!isValidArrival(arrival, pick)) {
      continue;
    }

    // compare sensor locations
    try {
      pick->time().value();
    } catch (...) {
      continue;
    }
    auto templateWfSensorLocation{
        Client::Inventory::Instance()->getSensorLocation(
            templateWfStreamId.netCode(), templateWfStreamId.staCode(),
            templateWfStreamId.locCode(), pick->time().value())};
    if (!templateWfSensorLocation) {
      auto msg{logPrefix +
               std::string{
                   "Sensor location not found in inventory for time: time="} +
               pick->time().value().iso()};

      SCDETECT_LOG_WARNING("%s", msg.c_str());
      throw builder::NoSensorLocation{msg};
    }
    pickWaveformId = pick->waveformID();
    auto pickWfSensorLocation{Client::Inventory::Instance()->getSensorLocation(
        pickWaveformId.networkCode(), pickWaveformId.stationCode(),
        pickWaveformId.locationCode(), pick->time().value())};
    if (!pickWfSensorLocation ||
        *templateWfSensorLocation != *pickWfSensorLocation) {
      continue;
    }

    break;
  }

  if (!pick) {
    arrival.reset();
    auto msg{logPrefix + std::string{"Failed to load pick: origin="} +
             _originId + std::string{", phase="} +
             streamConfig.templateConfig.phase};

    SCDETECT_LOG_WARNING("%s", msg.c_str());
    throw builder::BaseException{msg};
  }

  std::ostringstream oss;
  oss << utils::WaveformStreamID{pickWaveformId};
  SCDETECT_LOG_DEBUG(
      "%sUsing arrival pick: origin=%s, time=%s, phase=%s, stream=%s",
      logPrefix.c_str(), _originId.c_str(), pick->time().value().iso().c_str(),
      streamConfig.templateConfig.phase.c_str(), oss.str().c_str());

  auto wfStart{pick->time().value() +
               Core::TimeSpan{streamConfig.templateConfig.wfStart}};
  auto wfEnd{pick->time().value() +
             Core::TimeSpan{streamConfig.templateConfig.wfEnd}};

  // load stream metadata from inventory
  utils::WaveformStreamID wfStreamId{streamId};
  auto stream{Client::Inventory::Instance()->getStream(
      wfStreamId.netCode(), wfStreamId.staCode(), wfStreamId.locCode(),
      wfStreamId.chaCode(), wfStart)};

  if (!stream) {
    auto msg{logPrefix +
             std::string{"Stream not found in inventory for epoch: start="} +
             wfStart.iso() + std::string{", end="} + wfEnd.iso()};

    SCDETECT_LOG_WARNING("%s", msg.c_str());
    throw builder::NoStream{msg};
  }

  SCDETECT_LOG_DEBUG(
      "%sLoaded stream from inventory for epoch: start=%s, "
      "end=%s",
      logPrefix.c_str(), wfStart.iso().c_str(), wfEnd.iso().c_str());

  SCDETECT_LOG_DEBUG("Creating template processor (id=%s) ... ",
                     streamConfig.templateId.c_str());

  _product->_streamStates[streamId] = DetectorWaveformProcessor::StreamState{};

  // template related filter configuration (used for template waveform
  // processing)
  auto pickFilterId{pick->filterID()};
  auto templateWfFilterId{
      streamConfig.templateConfig.filter.value_or(pickFilterId)};
  utils::replaceEscapedXMLFilterIdChars(templateWfFilterId);

  std::unique_ptr<WaveformProcessor::Filter> rtTemplateFilter{nullptr};
  std::string rtFilterId{streamConfig.filter.value_or(pickFilterId)};
  utils::replaceEscapedXMLFilterIdChars(rtFilterId);
  // create template related filter (used during real-time stream
  // processing)
  if (!rtFilterId.empty()) {
    std::string err;
    rtTemplateFilter.reset(WaveformProcessor::Filter::Create(rtFilterId, &err));

    if (!rtTemplateFilter) {
      auto msg{logPrefix + "Compiling filter (" + rtFilterId +
               ") failed: " + err};

      SCDETECT_LOG_WARNING("%s", msg.c_str());
      throw builder::BaseException{msg};
    }
  }

  // prepare a demeaned waveform chunk (used for template waveform processor
  // configuration)
  auto margin{settings::kTemplateWaveformResampleMargin};
  if (!templateWfFilterId.empty()) {
    margin = std::max(margin, streamConfig.initTime);
  }
  Core::TimeSpan templateWfChunkMargin{margin};
  Core::Time templateWfChunkStartTime{wfStart - templateWfChunkMargin};
  Core::Time templateWfChunkEndTime{wfEnd + templateWfChunkMargin};

  WaveformHandlerIface::ProcessingConfig templateWfConfig;
  templateWfConfig.demean = true;

  GenericRecordCPtr templateWfChunk;
  try {
    templateWfChunk = wfHandler->get(
        templateWfStreamId.netCode(), templateWfStreamId.staCode(),
        templateWfStreamId.locCode(), templateWfStreamId.chaCode(),
        templateWfChunkStartTime, templateWfChunkEndTime, templateWfConfig);
  } catch (WaveformHandler::NoData &e) {
    throw builder::NoWaveformData{
        std::string{"Failed to load template waveform: "} + e.what()};
  } catch (std::exception &e) {
    throw builder::BaseException{
        std::string{"Failed to load template waveform: "} + e.what()};
  }

  // template processor
  auto templateProc{utils::make_unique<detector::TemplateWaveformProcessor>(
      templateWfChunk, templateWfFilterId, wfStart, wfEnd,
      streamConfig.templateId, _product.get())};

  templateProc->setFilter(rtTemplateFilter.release(), streamConfig.initTime);
  if (streamConfig.targetSamplingFrequency) {
    templateProc->setTargetSamplingFrequency(
        *streamConfig.targetSamplingFrequency);
  }
  templateProc->setDebugMode(debugMode);

  auto filterMsg{logPrefix + "Filters configured: filter=\"" + rtFilterId +
                 "\""};
  if (rtFilterId != templateWfFilterId) {
    filterMsg += " (template_filter=\"" + templateWfFilterId + "\")";
  }
  SCDETECT_LOG_DEBUG_PROCESSOR(templateProc, "%s", filterMsg.c_str());

  TemplateProcessorConfig c{std::move(templateProc),
                            streamConfig.mergingThreshold,
                            {stream->sensorLocation(), pick, arrival}};

  _processorConfigs.emplace(streamId, std::move(c));

  _arrivalPicks.push_back(detector::POT::ArrivalPick{arrival, pick});

  return *this;
}

void DetectorBuilder::finalize() {
  // use a POT to determine the max relative pick offset
  detector::PickOffsetTable pot{_arrivalPicks};

  // detector initialization time
  Core::TimeSpan po{pot.pickOffset().value_or(0)};
  if (po) {
    using pair_type = TemplateProcessorConfigs::value_type;
    const auto max{std::max_element(
        std::begin(_processorConfigs), std::end(_processorConfigs),
        [](const pair_type &lhs, const pair_type &rhs) {
          return lhs.second.processor->initTime() <
                 rhs.second.processor->initTime();
        })};

    _product->_initTime = (max->second.processor->initTime() > po
                               ? max->second.processor->initTime()
                               : po);

  } else if (pot.size()) {
    _product->_initTime =
        _processorConfigs.cbegin()->second.processor->initTime();
  } else {
    _product->disable();
  }

  const auto &cfg{_product->_config};
  _product->_detector.setTriggerThresholds(cfg.triggerOn, cfg.triggerOff);
  if (cfg.triggerDuration >= 0) {
    _product->_detector.enableTrigger(Core::TimeSpan{cfg.triggerDuration});
  }
  auto product{_product.get()};
  _product->_detector.setResultCallback(
      [product](const detector::Detector::Result &res) {
        product->storeDetection(res);
      });

  if (cfg.arrivalOffsetThreshold < 0) {
    _product->_detector.setArrivalOffsetThreshold(boost::none);
  } else {
    _product->_detector.setArrivalOffsetThreshold(cfg.arrivalOffsetThreshold);
  }

  if (cfg.minArrivals < 0) {
    _product->_detector.setMinArrivals(boost::none);
  } else {
    _product->_detector.setMinArrivals(cfg.minArrivals);
  }

  if (cfg.chunkSize < 0) {
    _product->_detector.setChunkSize(boost::none);
  } else {
    _product->_detector.setChunkSize(Core::TimeSpan{cfg.chunkSize});
  }

  auto bufferingOperator{
      utils::make_unique<waveform_operator::RingBufferOperator>(
          _product.get())};
  bufferingOperator->setGapThreshold(Core::TimeSpan{cfg.gapThreshold});
  bufferingOperator->setGapTolerance(Core::TimeSpan{cfg.gapTolerance});
  bufferingOperator->setGapInterpolation(cfg.gapInterpolation);

  std::unordered_set<std::string> usedPicks;
  for (auto &procConfigPair : _processorConfigs) {
    const auto &streamId{procConfigPair.first};
    auto &procConfig{procConfigPair.second};

    bufferingOperator->add(streamId,
                           Core::TimeSpan{std::max(30.0, cfg.chunkSize) *
                                          settings::kBufferMultiplicator});

    const auto &meta{procConfig.metadata};
    boost::optional<std::string> phase_hint;
    try {
      phase_hint = meta.pick->phaseHint();
    } catch (Core::ValueException &e) {
    }
    // initialize detection processing
    _product->_detector.add(
        std::move(procConfig.processor), bufferingOperator->get(streamId),
        streamId,
        detector::Arrival{
            {meta.pick->time().value(), meta.pick->waveformID(), phase_hint,
             meta.pick->time().value() - _product->_origin->time().value()},
            meta.arrival->phase(),
            meta.arrival->weight(),
        },
        detector::Detector::SensorLocation{
            meta.sensorLocation->latitude(), meta.sensorLocation->longitude(),
            meta.sensorLocation->station()->publicID()},
        procConfig.mergingThreshold);

    usedPicks.emplace(meta.pick->publicID());
  }
  _product->setOperator(bufferingOperator.release());

  // attach reference theoretical template arrivals to the product
  if (cfg.createTemplateArrivals) {
    std::ostringstream oss;
    for (size_t i = 0; i < _product->_origin->arrivalCount(); ++i) {
      const auto &arrival{_product->_origin->arrival(i)};
      const auto &pick{
          EventStore::Instance().get<DataModel::Pick>(arrival->pickID())};

      bool isDetectorArrival{usedPicks.find(arrival->pickID()) !=
                             usedPicks.end()};
      if (isDetectorArrival || arrival->phase().code().empty() ||
          !isValidArrival(arrival, pick)) {
        continue;
      }

      boost::optional<std::string> phaseHint;
      try {
        phaseHint = pick->phaseHint();
      } catch (Core::ValueException &e) {
      }
      boost::optional<double> lowerUncertainty;
      try {
        lowerUncertainty = pick->time().lowerUncertainty();
      } catch (Core::ValueException &e) {
      }
      boost::optional<double> upperUncertainty;
      try {
        upperUncertainty = pick->time().upperUncertainty();
      } catch (Core::ValueException &e) {
      }

      const utils::WaveformStreamID wfId{pick->waveformID()};
      oss << wfId;
      _product->_refTheoreticalTemplateArrivals.push_back(
          {{pick->time().value(), oss.str(), phaseHint,
            pick->time().value() - _product->_origin->time().value(),
            lowerUncertainty, upperUncertainty},
           arrival->phase(),
           arrival->weight()});
      oss.str("");
    }
  }
}

bool DetectorBuilder::isValidArrival(const DataModel::ArrivalCPtr arrival,
                                     const DataModel::PickCPtr pick) {
  if (!pick) {
    SCDETECT_LOG_DEBUG("Failed loading pick: %s", arrival->pickID().c_str());
    return false;
  }
  // check if both pick and arrival are properly configured
  try {
    if (pick->evaluationMode() != DataModel::MANUAL &&
        (arrival->weight() == 0 || !arrival->timeUsed())) {
      return false;
    }
  } catch (Core::ValueException &e) {
    return false;
  }
  return true;
}

const std::unordered_map<std::string, linker::MergingStrategy::Type>
    DetectorBuilder::_mergingStrategyLookupTable{
        {"all", linker::MergingStrategy::Type::kAll},
        {"greaterEqualTriggerOnThreshold",
         linker::MergingStrategy::Type::kGreaterEqualAssociationThres},
        {"greaterEqualMergingThreshold",
         linker::MergingStrategy::Type::kGreaterEqualMergingThres}};

}  // namespace detector
}  // namespace detect
}  // namespace Seiscomp
