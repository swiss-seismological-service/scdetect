#include "detectorwaveformprocessor.h"

#include "../log.h"
#include "../util/memory.h"
#include "detectorbuilder.h"

namespace Seiscomp {
namespace detect {
namespace detector {

DetectorWaveformProcessor::DetectorWaveformProcessor(
    const DataModel::OriginCPtr &origin)
    : _detector{origin}, _origin{origin} {}

DetectorBuilder DetectorWaveformProcessor::Create(const std::string &originId) {
  return DetectorBuilder(originId);
}

void DetectorWaveformProcessor::setFilter(Filter *filter,
                                          const Core::TimeSpan &initTime) {
  // XXX(damb): `DetectorWaveformProcessor` doesn't implement filter facilities
}

void DetectorWaveformProcessor::setResultCallback(
    const PublishDetectionCallback &callback) {
  _detectionCallback = callback;
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
    auto detection{util::make_smart<Detection>()};
    prepareDetection(detection, *_detection);
    emitDetection(nullptr, detection);

    _detection = boost::none;
  }
  WaveformProcessor::terminate();
}

const config::PublishConfig &DetectorWaveformProcessor::publishConfig() const {
  return _publishConfig;
}

processing::WaveformProcessor::StreamState &
DetectorWaveformProcessor::streamState(const Record *record) {
  return _streamStates.at(record->streamID());
}

void DetectorWaveformProcessor::process(StreamState &streamState,
                                        const Record *record,
                                        const DoubleArray &filteredData) {
  try {
    _detector.process(record);
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
      auto detection{util::make_smart<Detection>()};
      prepareDetection(detection, *_detection);
      emitDetection(record, detection);

      _detection = boost::none;
    }
  }
}

void DetectorWaveformProcessor::reset(StreamState &streamState) {
  // XXX(damb): drops all pending events
  _detector.reset();

  WaveformProcessor::reset(streamState);
}

bool DetectorWaveformProcessor::fill(processing::StreamState &streamState,
                                     const Record *record,
                                     DoubleArrayPtr &data) {
  // XXX(damb): `DetectorWaveformProcessor` does neither implement filtering
  // facilities nor does it perform a saturation check
  auto &s = dynamic_cast<WaveformProcessor::StreamState &>(streamState);
  s.receivedSamples += data->size();

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

  d->numChannelsAssociated = res.numChannelsAssociated;
  d->numChannelsUsed = res.numChannelsUsed;
  d->numStationsAssociated = res.numStationsAssociated;
  d->numStationsUsed = res.numStationsUsed;

  d->publishConfig.createArrivals = _publishConfig.createArrivals;
  d->publishConfig.createTemplateArrivals =
      _publishConfig.createTemplateArrivals;
  d->publishConfig.originMethodId = _publishConfig.originMethodId;
  d->publishConfig.createAmplitudes = _publishConfig.createAmplitudes;
  d->publishConfig.createMagnitudes = _publishConfig.createMagnitudes;

  if (_publishConfig.createTemplateArrivals) {
    for (const auto &arrival : _publishConfig.theoreticalTemplateArrivals) {
      auto theoreticalTemplateArrival{arrival};
      theoreticalTemplateArrival.pick.time =
          res.originTime + arrival.pick.offset + timeCorrection;
      d->publishConfig.theoreticalTemplateArrivals.push_back(
          theoreticalTemplateArrival);
    }
  }

  d->templateResults = res.templateResults;
  if (timeCorrection) {
    for (auto &templateResultPair : d->templateResults) {
      templateResultPair.second.arrival.pick.time += timeCorrection;
    }
  }
}
void DetectorWaveformProcessor::emitDetection(const Record *record,
                                              const DetectionCPtr &detection) {
  if (enabled() && _detectionCallback) {
    _detectionCallback(this, record, detection);
  }
}

}  // namespace detector
}  // namespace detect
}  // namespace Seiscomp
