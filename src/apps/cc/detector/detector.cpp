#include "detector.h"

#include <seiscomp/client/inventory.h>

#include "../eventstore.h"
#include "../log.h"
#include "../settings.h"
#include "../util/memory.h"
#include "../util/waveform_stream_id.h"
#include "linker/association.h"

namespace Seiscomp {
namespace detect {
namespace detector {

Detector::Builder::Builder(const std::string &originId) : _originId{originId} {
  DataModel::OriginCPtr origin{
      EventStore::Instance().getWithChildren<DataModel::Origin>(originId)};
  if (!origin) {
    SCDETECT_LOG_WARNING("Origin %s not found.", originId.c_str());
    throw builder::BaseException{std::string{"error while assigning origin: "} +
                                 originId};
  }

  // XXX(damb): Using `new` to access a non-public ctor; see also
  // https://abseil.io/tips/134
  setProduct(std::unique_ptr<Detector>(new Detector{origin}));
}

Detector::Builder &Detector::Builder::setId(const std::string &id) {
  product()->setId(id);
  product()->_detectorImpl.setId(id);
  return *this;
}

Detector::Builder &Detector::Builder::setConfig(
    const config::PublishConfig &publishConfig,
    const config::DetectorConfig &detectorConfig, bool playback) {
  product()->_publishConfig = publishConfig;

  product()->_config = detectorConfig;

  product()->_enabled = detectorConfig.enabled;

  product()->setGapThreshold(Core::TimeSpan{detectorConfig.gapThreshold});
  product()->setGapTolerance(Core::TimeSpan{detectorConfig.gapTolerance});
  product()->setGapInterpolation(detectorConfig.gapInterpolation);

  setMergingStrategy(detectorConfig.mergingStrategy);

  // configure playback related facilities
  if (playback) {
    product()->_detectorImpl.setMaxLatency(boost::none);
  } else {
    if (detectorConfig.maximumLatency > 0) {
      product()->_detectorImpl.setMaxLatency(
          Core::TimeSpan{detectorConfig.maximumLatency});
    }
  }

  return *this;
}

Detector::Builder &Detector::Builder::setStream(
    const std::string &streamId, const config::StreamConfig &streamConfig,
    WaveformHandlerIface *waveformHandler) {
  const auto &templateStreamId{streamConfig.templateConfig.wfStreamId};
  util::WaveformStreamID templateWfStreamId{templateStreamId};

  logging::TaggedMessage msg{streamId + " (" + templateStreamId + ")"};
  // configure pick from arrival
  DataModel::PickPtr pick;
  DataModel::WaveformStreamID pickWaveformId;
  DataModel::ArrivalPtr arrival;
  for (size_t i = 0; i < product()->_origin->arrivalCount(); ++i) {
    arrival = product()->_origin->arrival(i);

    if (arrival->phase().code() != streamConfig.templateConfig.phase) {
      continue;
    }

    pick = EventStore::Instance().get<DataModel::Pick>(arrival->pickID());
    if (!pick) {
      SCDETECT_LOG_DEBUG("Failed to load pick with id: %s",
                         arrival->pickID().c_str());
      continue;
    }
    if (!isValidArrival(*arrival, *pick)) {
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

  auto templateWaveformStartTime{
      pick->time().value() +
      Core::TimeSpan{streamConfig.templateConfig.wfStart}};
  auto templateWaveformEndTime{
      pick->time().value() + Core::TimeSpan{streamConfig.templateConfig.wfEnd}};

  // load stream metadata from inventory
  util::WaveformStreamID wfStreamId{streamId};
  auto *stream{Client::Inventory::Instance()->getStream(
      wfStreamId.netCode(), wfStreamId.staCode(), wfStreamId.locCode(),
      wfStreamId.chaCode(), templateWaveformStartTime)};

  if (!stream) {
    msg.setText("failed to load stream from inventory: start=" +
                templateWaveformStartTime.iso() +
                ", end=" + templateWaveformEndTime.iso());
    throw builder::NoStream{logging::to_string(msg)};
  }

  msg.setText("loaded stream from inventory for epoch: start=" +
              templateWaveformStartTime.iso() +
              ", end=" + templateWaveformEndTime.iso());
  SCDETECT_LOG_DEBUG("%s", logging::to_string(msg).c_str());

  const auto templateWaveformProcessorId{
      product()->id().empty() ? streamConfig.templateId
                              : product()->id() + settings::kProcessorIdSep +
                                    streamConfig.templateId};
  msg.setText("creating template waveform processor with id: " +
              templateWaveformProcessorId);
  SCDETECT_LOG_DEBUG("%s", logging::to_string(msg).c_str());

  product()->_streamStates[streamId] = Detector::StreamState{};

  // template related filter configuration (used for template waveform
  // processing)
  TemplateWaveform::ProcessingConfig processingConfig;
  processingConfig.templateStartTime = templateWaveformStartTime;
  processingConfig.templateEndTime = templateWaveformEndTime;
  processingConfig.safetyMargin = settings::kTemplateWaveformResampleMargin;
  processingConfig.detrend = false;
  processingConfig.demean = true;

  auto pickFilterId{pick->filterID()};
  auto templateWfFilterId{
      streamConfig.templateConfig.filter.value_or(pickFilterId)};
  if (!templateWfFilterId.empty()) {
    util::replaceEscapedXMLFilterIdChars(templateWfFilterId);
    processingConfig.filter = templateWfFilterId;
    processingConfig.initTime = Core::TimeSpan{streamConfig.initTime};
  }

  // template waveform processor
  std::unique_ptr<detector::TemplateWaveformProcessor>
      templateWaveformProcessor;
  try {
    auto templateWaveform{TemplateWaveform::load(
        waveformHandler, templateWfStreamId.netCode(),
        templateWfStreamId.staCode(), templateWfStreamId.locCode(),
        templateWfStreamId.chaCode(), processingConfig)};

    templateWaveform.setReferenceTime(pick->time().value());

    templateWaveformProcessor =
        util::make_unique<detector::TemplateWaveformProcessor>(
            templateWaveform);
  } catch (WaveformHandler::NoData &e) {
    msg.setText("failed to load template waveform: " + std::string{e.what()});
    throw builder::NoWaveformData{logging::to_string(msg)};
  }

  templateWaveformProcessor->setId(templateWaveformProcessorId);

  std::string rtFilterId{streamConfig.filter.value_or(pickFilterId)};
  // configure template related filter (used during real-time stream
  // processing)
  if (!rtFilterId.empty()) {
    util::replaceEscapedXMLFilterIdChars(rtFilterId);
    try {
      templateWaveformProcessor->setFilter(processing::createFilter(rtFilterId),
                                           streamConfig.initTime);
    } catch (processing::WaveformProcessor::BaseException &e) {
      msg.setText(e.what());
      throw builder::BaseException{logging::to_string(msg)};
    }
  }

  if (streamConfig.targetSamplingFrequency) {
    templateWaveformProcessor->setTargetSamplingFrequency(
        *streamConfig.targetSamplingFrequency);
  }

  std::string text{"filters configured: filter=\"" + rtFilterId + "\""};
  if (rtFilterId != templateWfFilterId) {
    text += " (template_filter=\"" + templateWfFilterId + "\")";
  }
  msg.setText(text);
  SCDETECT_LOG_DEBUG_PROCESSOR(templateWaveformProcessor, "%s",
                               logging::to_string(msg).c_str());

  TemplateProcessorConfig c{std::move(templateWaveformProcessor),
                            streamConfig.mergingThreshold,
                            {stream->sensorLocation(), pick, arrival}};

  _processorConfigs.emplace(streamId, std::move(c));

  return *this;
}

void Detector::Builder::finalize() {
  auto hasNoChildren{_processorConfigs.empty()};
  if (hasNoChildren) {
    product()->disable();
  }

  product()->_initTime = Core::TimeSpan{0.0};

  const auto &cfg{product()->_config};
  product()->_detectorImpl.setTriggerThresholds(cfg.triggerOn, cfg.triggerOff);
  if (cfg.triggerDuration >= 0) {
    product()->_detectorImpl.enableTrigger(Core::TimeSpan{cfg.triggerDuration});
  }
  auto productCopy{product()};
  product()->_detectorImpl.setResultCallback(
      [productCopy](const detector::DetectorImpl::Result &res) {
        productCopy->storeDetection(res);
      });

  if (cfg.arrivalOffsetThreshold < 0) {
    product()->_detectorImpl.setArrivalOffsetThreshold(boost::none);
  } else {
    product()->_detectorImpl.setArrivalOffsetThreshold(
        Core::TimeSpan{cfg.arrivalOffsetThreshold});
  }

  if (cfg.minArrivals < 0) {
    product()->_detectorImpl.setMinArrivals(boost::none);
  } else {
    product()->_detectorImpl.setMinArrivals(cfg.minArrivals);
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

    procConfig.processor->setGapThreshold(
        Core::TimeSpan{product()->_config.gapThreshold});
    procConfig.processor->setGapTolerance(
        Core::TimeSpan{product()->_config.gapTolerance});
    procConfig.processor->setGapInterpolation(
        product()->_config.gapInterpolation);

    // initialize detection processing
    product()->_detectorImpl.add(
        std::move(procConfig.processor), streamId,
        detector::Arrival{
            {meta.pick->time().value(), meta.pick->waveformID(), phase_hint,
             meta.pick->time().value() - product()->_origin->time().value()},
            meta.arrival->phase(),
            meta.arrival->weight(),
        },
        detector::DetectorImpl::SensorLocation{
            meta.sensorLocation->latitude(), meta.sensorLocation->longitude(),
            meta.sensorLocation->station()->publicID()},
        procConfig.mergingThreshold);

    usedPicks.emplace(meta.pick->publicID());
  }

  // attach reference theoretical template arrivals to the product
  if (product()->_publishConfig.createTemplateArrivals) {
    for (size_t i = 0; i < product()->_origin->arrivalCount(); ++i) {
      const auto &arrival{product()->_origin->arrival(i)};
      const auto &pick{
          EventStore::Instance().get<DataModel::Pick>(arrival->pickID())};

      bool isDetectorArrival{usedPicks.find(arrival->pickID()) !=
                             usedPicks.end()};
      if (isDetectorArrival || arrival->phase().code().empty()) {
        continue;
      }

      if (!pick) {
        SCDETECT_LOG_DEBUG("Failed to load pick with id: %s",
                           arrival->pickID().c_str());
        continue;
      }

      if (!isValidArrival(*arrival, *pick)) {
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

      product()->_publishConfig.theoreticalTemplateArrivals.push_back(
          {{pick->time().value(),
            util::to_string(util::WaveformStreamID{pick->waveformID()}),
            phaseHint,
            pick->time().value() - product()->_origin->time().value(),
            lowerUncertainty, upperUncertainty},
           arrival->phase(),
           arrival->weight()});
    }
  }
}

void Detector::Builder::setMergingStrategy(const std::string &strategyId) {
  if ("all" == strategyId) {
    product()->_detectorImpl.setMergingStrategy(
        [](const linker::Association::TemplateResult &result,
           double associationThreshold,
           double mergingThreshold) { return true; });
  } else if ("greaterEqualTriggerOnThreshold" == strategyId) {
    product()->_detectorImpl.setMergingStrategy(
        [](const linker::Association::TemplateResult &result,
           double associationThreshold, double mergingThreshold) {
          return result.resultIt->coefficient >= associationThreshold;
        });

  } else if ("greaterEqualMergingThreshold" == strategyId) {
    product()->_detectorImpl.setMergingStrategy(
        [](const linker::Association::TemplateResult &result,
           double associationThreshold, double mergingThreshold) {
          return result.resultIt->coefficient >= mergingThreshold;
        });
  } else {
    throw builder::BaseException{"invalid merging strategy: " + strategyId};
  }
}

bool Detector::Builder::isValidArrival(const DataModel::Arrival &arrival,
                                       const DataModel::Pick &pick) {
  // check if both pick and arrival are properly configured
  try {
    if (pick.evaluationMode() != DataModel::MANUAL &&
        (arrival.weight() == 0 || !arrival.timeUsed())) {
      return false;
    }
  } catch (Core::ValueException &e) {
    return false;
  }
  return true;
}

/* ------------------------------------------------------------------------- */
Detector::Detector(const DataModel::OriginCPtr &origin)
    : _detectorImpl{origin}, _origin{origin} {}

Detector::Builder Detector::Create(const std::string &originId) {
  return Builder(originId);
}

void Detector::setGapInterpolation(bool gapInterpolation) {
  processing::WaveformProcessor::setGapInterpolation(gapInterpolation);
  _detectorImpl.setGapInterpolation(gapInterpolation);
}

void Detector::setGapThreshold(const Core::TimeSpan &duration) {
  processing::WaveformProcessor::setGapThreshold(duration);
  _detectorImpl.setGapThreshold(duration);
}

void Detector::setGapTolerance(const Core::TimeSpan &duration) {
  processing::WaveformProcessor::setGapTolerance(duration);
  _detectorImpl.setGapTolerance(duration);
}

void Detector::setResultCallback(const PublishDetectionCallback &callback) {
  _detectionCallback = callback;
}

void Detector::reset() {
  SCDETECT_LOG_DEBUG_PROCESSOR(this, "Resetting detector ...");

  // reset template (child) related facilities
  for (auto &streamStatePair : _streamStates) {
    streamStatePair.second = WaveformProcessor::StreamState{};
  }

  _detectorImpl.reset();

  WaveformProcessor::reset();
}

void Detector::terminate() {
  SCDETECT_LOG_DEBUG_PROCESSOR(this, "Terminating ...");

  _detectorImpl.flush();
  processDetections(nullptr);

  WaveformProcessor::terminate();
}

const config::PublishConfig &Detector::publishConfig() const {
  return _publishConfig;
}

const TemplateWaveformProcessor *Detector::processor(
    const std::string &processorId) const {
  return _detectorImpl.processor(processorId);
}

processing::WaveformProcessor::StreamState *Detector::streamState(
    const Record *record) {
  return &_streamStates.at(record->streamID());
}

void Detector::process(StreamState &streamState, const Record *record,
                       const DoubleArray &filteredData) {
  try {
    _detectorImpl.feed(record);
  } catch (detector::DetectorImpl::ProcessingError &e) {
    SCDETECT_LOG_WARNING_PROCESSOR(this, "%s: %s. Resetting.",
                                   record->streamID().c_str(), e.what());
    _detectorImpl.reset();
  } catch (std::exception &e) {
    SCDETECT_LOG_ERROR_PROCESSOR(this, "%s: unhandled exception: %s",
                                 record->streamID().c_str(), e.what());

    setStatus(WaveformProcessor::Status::kError, 0);
  } catch (...) {
    SCDETECT_LOG_ERROR_PROCESSOR(this, "%s: unknown exception",
                                 record->streamID().c_str());

    setStatus(WaveformProcessor::Status::kError, 0);
  }

  if (!finished()) {
    processDetections(record);
  }
}

bool Detector::store(const Record *record) {
  processing::WaveformProcessor::store(record);

  return !finished();
}

void Detector::reset(StreamState &streamState) {
  // XXX(damb): drops all pending events
  _detectorImpl.reset();

  WaveformProcessor::reset(streamState);
}

bool Detector::fill(processing::StreamState &streamState, const Record *record,
                    DoubleArrayPtr &data) {
  // XXX(damb): `Detector` does neither implement filtering facilities nor does
  // it perform a saturation check
  auto &s = dynamic_cast<WaveformProcessor::StreamState &>(streamState);
  s.receivedSamples += data->size();

  return true;
}

bool Detector::handleGap(processing::StreamState &streamState,
                         const Record *record, DoubleArrayPtr &data) {
  // XXX(damb): do not perform any gap handling. Instead, the underlying
  // `TemplateWaveformProcessor`s are performing the gap handling by
  // themselves.
  return true;
}

void Detector::storeDetection(const detector::DetectorImpl::Result &res) {
  _detectionQueue.emplace_back(res);
}

std::unique_ptr<const Detector::Detection> Detector::createDetection(
    const detector::DetectorImpl::Result &res) {
  const Core::TimeSpan timeCorrection{_config.timeCorrection};

  auto ret{util::make_unique<Detection>()};

  ret->score = res.score;
  ret->time = res.originTime + timeCorrection;
  ret->latitude = _origin->latitude().value();
  ret->longitude = _origin->longitude().value();
  ret->depth = _origin->depth().value();

  ret->numChannelsAssociated = res.numChannelsAssociated;
  ret->numChannelsUsed = res.numChannelsUsed;
  ret->numStationsAssociated = res.numStationsAssociated;
  ret->numStationsUsed = res.numStationsUsed;

  ret->publishConfig.createArrivals = _publishConfig.createArrivals;
  ret->publishConfig.createTemplateArrivals =
      _publishConfig.createTemplateArrivals;
  ret->publishConfig.originMethodId = _publishConfig.originMethodId;
  ret->publishConfig.createAmplitudes = _publishConfig.createAmplitudes;
  ret->publishConfig.createMagnitudes = _publishConfig.createMagnitudes;

  if (_publishConfig.createTemplateArrivals) {
    for (const auto &arrival : _publishConfig.theoreticalTemplateArrivals) {
      auto theoreticalTemplateArrival{arrival};
      theoreticalTemplateArrival.pick.time =
          res.originTime + arrival.pick.offset + timeCorrection;
      ret->publishConfig.theoreticalTemplateArrivals.push_back(
          theoreticalTemplateArrival);
    }
  }

  ret->templateResults = res.templateResults;
  if (timeCorrection) {
    for (auto &templateResultPair : ret->templateResults) {
      templateResultPair.second.arrival.pick.time += timeCorrection;
    }
  }

  return ret;
}

void Detector::emitDetection(const Record *record,
                             std::unique_ptr<const Detection> detection) {
  if (enabled() && _detectionCallback) {
    _detectionCallback(this, record, std::move(detection));
  }
}

void Detector::processDetections(const Record *record) {
  while (!_detectionQueue.empty()) {
    emitDetection(record, createDetection(_detectionQueue.front()));
    _detectionQueue.pop_front();
  }
}

}  // namespace detector
}  // namespace detect
}  // namespace Seiscomp
