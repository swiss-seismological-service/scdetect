#include "waveform_processor.h"

#include <exception>

#include "waveform_operator.h"

namespace Seiscomp {
namespace detect {
namespace processing {

WaveformProcessor::Result::~Result() {}

void WaveformProcessor::enable() {
  if (_enabled) return;
  _enabled = true;
}

void WaveformProcessor::disable() {
  if (!_enabled) return;
  _enabled = false;
}

bool WaveformProcessor::enabled() const { return _enabled; }

void WaveformProcessor::setResultCallback(
    const PublishResultCallback &callback) {
  _resultCallback = callback;
}

void WaveformProcessor::setSaturationThreshold(
    const boost::optional<double> &threshold) {
  _saturationThreshold = threshold;
}

WaveformProcessor::Status WaveformProcessor::status() const { return _status; }

double WaveformProcessor::statusValue() const { return _statusValue; }

void WaveformProcessor::setOperator(WaveformOperator *op) {
  if (_waveformOperator) {
    _waveformOperator.reset();
  }

  _waveformOperator.reset(op);
  if (_waveformOperator) {
    _waveformOperator->setStoreCallback(
        [this](const Record *record) { return store(record); });
  }
}

const Core::TimeSpan WaveformProcessor::initTime() const { return _initTime; }

bool WaveformProcessor::finished() const {
  return Status::kInProgress < _status;
}

bool WaveformProcessor::feed(const Record *record) {
  if (record->sampleCount() == 0) return false;

  if (!_waveformOperator) {
    return store(record);
  }

  WaveformProcessor::Status s{_waveformOperator->feed(record)};
  if (s > WaveformProcessor::Status::kTerminated) {
    setStatus(s, -1);
    return false;
  }
  return true;
}

void WaveformProcessor::reset() {
  if (_waveformOperator) {
    _waveformOperator->reset();
  }

  _status = Status::kWaitingForData;
  _statusValue = 0;
}

void WaveformProcessor::terminate() {
  setStatus(Status::kTerminated, static_cast<int>(_status));
}

void WaveformProcessor::close() const {}

WaveformProcessor::StreamState::~StreamState() {
  if (filter) {
    delete filter;
  }
}

bool WaveformProcessor::store(const Record *record) {
  if (WaveformProcessor::Status::kInProgress < status() || !record->data())
    return false;

  try {
    StreamState &currentStreamState{streamState(record)};

    DoubleArrayPtr data{
        dynamic_cast<DoubleArray *>(record->data()->copy(Array::DOUBLE))};

    if (currentStreamState.lastRecord) {
      if (record == currentStreamState.lastRecord) {
        return false;
      } else if (record->samplingFrequency() !=
                 currentStreamState.samplingFrequency) {
        SCDETECT_LOG_WARNING_PROCESSOR(
            this,
            "%s: sampling frequency changed, resetting stream (sfreq_record != "
            "sfreq_stream): %f != %f",
            record->streamID().c_str(), record->samplingFrequency(),
            currentStreamState.samplingFrequency);

        reset(currentStreamState);
      } else if (!handleGap(currentStreamState, record, data)) {
        return false;
      }

      currentStreamState.dataTimeWindow.setEndTime(record->endTime());
    }

    if (!currentStreamState.lastRecord) {
      try {
        setupStream(currentStreamState, record);
      } catch (std::exception &e) {
        SCDETECT_LOG_WARNING_PROCESSOR(this, "%s: Failed to setup stream: %s",
                                       record->streamID().c_str(), e.what());
        return false;
      }
    }
    currentStreamState.lastSample = (*data)[data->size() - 1];

    fill(currentStreamState, record, data);
    if (Status::kInProgress < status()) return false;

    processIfEnoughDataReceived(currentStreamState, record, *data);

    currentStreamState.lastRecord = record;

  } catch (...) {
    return false;
  }
  return true;
}

void WaveformProcessor::reset(StreamState &streamState) {
  streamState.lastRecord.reset();
}

bool WaveformProcessor::fill(processing::StreamState &streamState,
                             const Record *record, DoubleArrayPtr &data) {
  auto &s = dynamic_cast<WaveformProcessor::StreamState &>(streamState);

  const auto n{static_cast<size_t>(data->size())};
  s.receivedSamples += n;

  if (_saturationThreshold && checkIfSaturated(data)) {
    return false;
  }

  if (s.filter) {
    auto samples{data->typedData()};
    s.filter->apply(n, samples);
  }

  return true;
}

bool WaveformProcessor::checkIfSaturated(DoubleArrayPtr &data) {
  const auto samples{data->typedData()};
  for (int i = 0; i < data->size(); ++i) {
    if (fabs(samples[i]) >= *_saturationThreshold) {
      setStatus(Status::kDataClipped, samples[i]);
      return true;
    }
  }

  return false;
}

bool WaveformProcessor::processIfEnoughDataReceived(
    StreamState &streamState, const Record *record,
    const DoubleArray &filteredData) {
  bool processed{false};
  if (!streamState.initialized) {
    if (enoughDataReceived(streamState)) {
      // streamState.initialized = true;
      process(streamState, record, filteredData);
      // NOTE: To allow derived classes to notice modification of the variable
      // streamState.initialized, it is necessary to set this after calling
      // process().
      streamState.initialized = true;
      processed = true;
    }
  } else {
    // Call process to cause a derived processor to work on the data.
    process(streamState, record, filteredData);
    processed = true;
  }
  return processed;
}

bool WaveformProcessor::enoughDataReceived(
    const StreamState &streamState) const {
  return streamState.receivedSamples >= streamState.neededSamples;
}

void WaveformProcessor::emitResult(const Record *record,
                                   const ResultCPtr &result) {
  if (enabled() && _resultCallback) _resultCallback(this, record, result);
}

void WaveformProcessor::setupStream(StreamState &streamState,
                                    const Record *record) {
  const auto &f{record->samplingFrequency()};
  streamState.samplingFrequency = f;

  if (gapInterpolation()) {
    setMinimumGapThreshold(streamState, record, id());
  }

  streamState.neededSamples = static_cast<size_t>(_initTime * f + 0.5);
  if (streamState.filter) {
    streamState.filter->setSamplingFrequency(f);
  }

  // update the received data timewindow
  streamState.dataTimeWindow = record->timeWindow();

  if (streamState.filter) {
    streamState.filter->setStartTime(record->startTime());
    streamState.filter->setStreamID(
        record->networkCode(), record->stationCode(), record->locationCode(),
        record->channelCode());
  }
}

void WaveformProcessor::setStatus(Status status, double value) {
  _status = status;
  _statusValue = value;
}

std::unique_ptr<WaveformProcessor::Filter> createFilter(
    const std::string &filter) {
  std::string err;
  std::unique_ptr<WaveformProcessor::Filter> ret{
      WaveformProcessor::Filter::Create(filter, &err)};
  if (!ret) {
    throw WaveformProcessor::BaseException{"failed to compile filter (" +
                                           filter + "): " + err};
  }
  return ret;
}

}  // namespace processing
}  // namespace detect
}  // namespace Seiscomp
