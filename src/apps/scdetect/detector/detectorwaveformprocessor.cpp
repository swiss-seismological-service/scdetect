#include "detectorwaveformprocessor.h"

#include "../log.h"

namespace Seiscomp {
namespace detect {
namespace detector {

DetectorWaveformProcessor::DetectorWaveformProcessor(
    const std::string &id, const DataModel::OriginCPtr &origin)
    : WaveformProcessor{id}, _detector{this, origin}, _origin{origin} {}

DetectorBuilder DetectorWaveformProcessor::Create(const std::string &detectorId,
                                                  const std::string &originId) {
  return DetectorBuilder(detectorId, originId);
}

void DetectorWaveformProcessor::setFilter(Filter *filter,
                                          const Core::TimeSpan &initTime) {
  // XXX(damb): `DetectorWaveformProcessor` doesn't implement filter facilities
}

const Core::TimeWindow &DetectorWaveformProcessor::processed() const {
  return _detector.processed();
}

void DetectorWaveformProcessor::reset() {
  SCDETECT_LOG_DEBUG_PROCESSOR(this, "Resetting detector ...");

  // reset template (child) related facilities
  for (auto &streamStatePair : _streamStates) {
    streamStatePair.second = WaveformProcessor::StreamState{};
  }

  _detector.reset();

  WaveformProcessor::reset();
}

void DetectorWaveformProcessor::terminate() {
  SCDETECT_LOG_DEBUG_PROCESSOR(this, "Terminating ...");

  _detector.terminate();
  if (_detection) {
    auto detection{utils::make_smart<Detection>()};
    prepareDetection(detection, *_detection);
    emitResult(nullptr, detection);

    _detection = boost::none;
  }
  WaveformProcessor::terminate();
}

WaveformProcessor::StreamState &DetectorWaveformProcessor::streamState(
    const Record *record) {
  return _streamStates.at(record->streamID());
}

void DetectorWaveformProcessor::process(StreamState &streamState,
                                        const Record *record,
                                        const DoubleArray &filteredData) {
  try {
    _detector.process(record->streamID());
  } catch (detector::Detector::ProcessingError &e) {
    SCDETECT_LOG_WARNING_PROCESSOR(this, "%s: %s. Resetting.",
                                   record->streamID().c_str(), e.what());
    _detector.reset();
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
    if (_detection) {
      auto detection{utils::make_smart<Detection>()};
      prepareDetection(detection, *_detection);
      emitResult(record, detection);

      _detection = boost::none;
    }
  }
}

void DetectorWaveformProcessor::reset(StreamState &streamState,
                                      const Record *record) {
  // XXX(damb): drops all pending events
  _detector.reset();

  WaveformProcessor::reset(streamState, record);
}

void DetectorWaveformProcessor::fill(StreamState &streamState,
                                     const Record *record,
                                     DoubleArrayPtr &data) {
  // XXX(damb): `DetectorWaveformProcessor` does not implement filtering
  // facilities
  streamState.receivedSamples += data->size();
}

bool DetectorWaveformProcessor::enoughDataReceived(
    const StreamState &streamState) const {
  for (const auto &streamStatePair : _streamStates) {
    const auto &state{streamStatePair.second};
    if (state.receivedSamples <= state.neededSamples) {
      return false;
    }
  }
  return true;
}

void DetectorWaveformProcessor::storeDetection(
    const detector::Detector::Result &res) {
  _detection = res;
}

void DetectorWaveformProcessor::prepareDetection(
    DetectionPtr &d, const detector::Detector::Result &res) {
  const Core::TimeSpan timeCorrection{_config.timeCorrection};

  d->fit = _detection.value().fit;
  d->time = res.originTime + timeCorrection;
  d->latitude = _origin->latitude().value();
  d->longitude = _origin->longitude().value();
  d->depth = _origin->depth().value();

  const auto &mag{res.magnitude};
  d->magnitude = mag.value_or(_magnitude->magnitude().value());

  d->numChannelsAssociated = res.numChannelsAssociated;
  d->numChannelsUsed = res.numChannelsUsed;
  d->numStationsAssociated = res.numStationsAssociated;
  d->numStationsUsed = res.numStationsUsed;

  d->withArrivals = _config.createArrivals;
  d->templateResults = res.templateResults;

  if (timeCorrection) {
    for (auto &templateResultPair : d->templateResults) {
      templateResultPair.second.arrival.pick.time += timeCorrection;
    }
  }

  if (_config.createTemplateArrivals) {
    for (const auto &arrival : _refTheoreticalTemplateArrivals) {
      auto theoreticalTemplateArrival{arrival};
      theoreticalTemplateArrival.pick.time =
          res.originTime + arrival.pick.offset + timeCorrection;
      d->theoreticalTemplateArrivals.push_back(theoreticalTemplateArrival);
    }
  }
}

}  // namespace detector
}  // namespace detect
}  // namespace Seiscomp