#include "detectorbuilder.h"

#include <seiscomp/client/inventory.h>
#include <seiscomp/core/datetime.h>
#include <seiscomp/core/exceptions.h>
#include <seiscomp/core/timewindow.h>

#include <boost/optional/optional.hpp>
#include <cmath>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>

#include "../eventstore.h"
#include "../log.h"
#include "../settings.h"
#include "../util/memory.h"
#include "../util/util.h"
#include "../util/waveform_stream_id.h"
#include "detectorwaveformprocessor.h"

namespace Seiscomp {
namespace detect {
namespace detector {

DetectorBuilder::DetectorBuilder(const std::string &originId)
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
      new DetectorWaveformProcessor{origin});
}

DetectorBuilder &DetectorBuilder::setId(const std::string &id) {
  _product->setId(id);
  _product->_detector.setId(id);
  return *this;
}

DetectorBuilder &DetectorBuilder::setConfig(
    const config::PublishConfig &publishConfig,
    const config::DetectorConfig &detectorConfig, bool playback) {
  _product->_publishConfig = publishConfig;

  _product->_config = detectorConfig;

  _product->_enabled = detectorConfig.enabled;

  _product->setGapThreshold(Core::TimeSpan{detectorConfig.gapThreshold});
  _product->setGapTolerance(Core::TimeSpan{detectorConfig.gapTolerance});
  _product->setGapInterpolation(detectorConfig.gapInterpolation);

  _product->_detector.setMergingStrategy(
      _mergingStrategyLookupTable.at(detectorConfig.mergingStrategy));

  // configure playback related facilities
  if (playback) {
    _product->_detector.setMaxLatency(boost::none);
  } else {
    if (detectorConfig.maximumLatency > 0) {
      _product->_detector.setMaxLatency(
          Core::TimeSpan{detectorConfig.maximumLatency});
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

  return *this;
}

DetectorBuilder &DetectorBuilder::setStream(
    const std::string &streamId, const config::StreamConfig &streamConfig,
    WaveformHandlerIface *waveformHandler) {
  const auto &templateStreamId{streamConfig.templateConfig.wfStreamId};
  util::WaveformStreamID templateWfStreamId{templateStreamId};

  logging::TaggedMessage msg{streamId + " (" + templateStreamId + ")"};
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
      msg.setText("sensor location not found in inventory for time: " +
                  pick->time().value().iso());
      throw builder::NoSensorLocation{logging::to_string(msg)};
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
    msg.setText("failed to load pick: origin=" + _originId +
                ", phase=" + streamConfig.templateConfig.phase);
    throw builder::NoPick{logging::to_string(msg)};
  }

  msg.setText("using arrival pick: origin=" + _originId +
              ", time=" + pick->time().value().iso() +
              ", phase=" + streamConfig.templateConfig.phase + ", stream=" +
              util::to_string(util::WaveformStreamID{pickWaveformId}));
  SCDETECT_LOG_DEBUG("%s", logging::to_string(msg).c_str());

  auto wfStart{pick->time().value() +
               Core::TimeSpan{streamConfig.templateConfig.wfStart}};
  auto wfEnd{pick->time().value() +
             Core::TimeSpan{streamConfig.templateConfig.wfEnd}};

  // load stream metadata from inventory
  util::WaveformStreamID wfStreamId{streamId};
  auto stream{Client::Inventory::Instance()->getStream(
      wfStreamId.netCode(), wfStreamId.staCode(), wfStreamId.locCode(),
      wfStreamId.chaCode(), wfStart)};

  if (!stream) {
    msg.setText("failed to load stream from inventory: start=" + wfStart.iso() +
                ", end=" + wfEnd.iso());
    throw builder::NoStream{logging::to_string(msg)};
  }

  msg.setText("loaded stream from inventory for epoch: start=" + wfStart.iso() +
              ", end=" + wfEnd.iso());
  SCDETECT_LOG_DEBUG("%s", logging::to_string(msg).c_str());

  msg.setText("creating template waveform processor with id: " +
              streamConfig.templateId);
  SCDETECT_LOG_DEBUG("%s", logging::to_string(msg).c_str());

  _product->_streamStates[streamId] = DetectorWaveformProcessor::StreamState{};

  // template related filter configuration (used for template waveform
  // processing)
  auto pickFilterId{pick->filterID()};
  auto templateWfFilterId{
      streamConfig.templateConfig.filter.value_or(pickFilterId)};
  util::replaceEscapedXMLFilterIdChars(templateWfFilterId);

  std::unique_ptr<WaveformProcessor::Filter> rtTemplateFilter{nullptr};
  std::string rtFilterId{streamConfig.filter.value_or(pickFilterId)};
  util::replaceEscapedXMLFilterIdChars(rtFilterId);
  // create template related filter (used during real-time stream
  // processing)
  if (!rtFilterId.empty()) {
    std::string err;
    rtTemplateFilter.reset(WaveformProcessor::Filter::Create(rtFilterId, &err));

    if (!rtTemplateFilter) {
      msg.setText("compiling filter (" + rtFilterId + ") failed: " + err);
      throw builder::BaseException{logging::to_string(msg)};
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
    templateWfChunk = waveformHandler->get(
        templateWfStreamId.netCode(), templateWfStreamId.staCode(),
        templateWfStreamId.locCode(), templateWfStreamId.chaCode(),
        templateWfChunkStartTime, templateWfChunkEndTime, templateWfConfig);
  } catch (WaveformHandler::NoData &e) {
    msg.setText("failed to load template waveform: " + std::string{e.what()});
    throw builder::NoWaveformData{logging::to_string(msg)};
  } catch (std::exception &e) {
    msg.setText("failed to load template waveform: " + std::string{e.what()});
    throw builder::BaseException{logging::to_string(msg)};
  }

  // template processor
  auto templateProc{util::make_unique<detector::TemplateWaveformProcessor>(
      templateWfChunk, templateWfFilterId, wfStart, wfEnd, _product.get())};

  templateProc->setId(_product->id().empty()
                          ? streamConfig.templateId
                          : _product->id() + settings::kProcessorIdSep +
                                streamConfig.templateId);

  templateProc->setFilter(rtTemplateFilter.release(), streamConfig.initTime);
  if (streamConfig.targetSamplingFrequency) {
    templateProc->setTargetSamplingFrequency(
        *streamConfig.targetSamplingFrequency);
  }

  std::string text{"filters configured: filter=\"" + rtFilterId + "\""};
  if (rtFilterId != templateWfFilterId) {
    text += " (template_filter=\"" + templateWfFilterId + "\")";
  }
  msg.setText(text);
  SCDETECT_LOG_DEBUG_PROCESSOR(templateProc, "%s",
                               logging::to_string(msg).c_str());

  TemplateProcessorConfig c{std::move(templateProc),
                            streamConfig.mergingThreshold,
                            {stream->sensorLocation(), pick, arrival}};

  _processorConfigs.emplace(streamId, std::move(c));

  _arrivalPicks.push_back(detector::POT::ArrivalPick{arrival, pick});

  return *this;
}

void DetectorBuilder::finalize() {
  // use a POT to determine the max relative pick offset
  auto hasNoChildren{_processorConfigs.empty()};
  if (hasNoChildren) {
    _product->disable();
  }

  _product->_initTime = Core::TimeSpan{0.0};

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

  std::unordered_set<std::string> usedPicks;
  for (auto &procConfigPair : _processorConfigs) {
    const auto &streamId{procConfigPair.first};
    auto &procConfig{procConfigPair.second};

    const auto &meta{procConfig.metadata};
    boost::optional<std::string> phase_hint;
    try {
      phase_hint = meta.pick->phaseHint();
    } catch (Core::ValueException &e) {
    }
    // initialize detection processing
    _product->_detector.add(
        std::move(procConfig.processor), streamId,
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

  // attach reference theoretical template arrivals to the product
  if (_product->_publishConfig.createTemplateArrivals) {
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

      _product->_publishConfig.theoreticalTemplateArrivals.push_back(
          {{pick->time().value(),
            util::to_string(util::WaveformStreamID{pick->waveformID()}),
            phaseHint, pick->time().value() - _product->_origin->time().value(),
            lowerUncertainty, upperUncertainty},
           arrival->phase(),
           arrival->weight()});
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
