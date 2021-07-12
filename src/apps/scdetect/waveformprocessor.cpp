#include "waveformprocessor.h"

#include <exception>

#include "log.h"
#include "utils.h"
#include "waveformoperator.h"

namespace Seiscomp {
namespace detect {

WaveformProcessor::WaveformProcessor(
    const std::string &id, const boost::filesystem::path &debugInfoDir)
    : Processor{id}, _debugInfoDir{debugInfoDir} {}

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

const boost::filesystem::path &WaveformProcessor::debugInfoDir() const {
  return _debugInfoDir;
}

bool WaveformProcessor::debugMode() const { return !_debugInfoDir.empty(); }

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

std::string WaveformProcessor::debugString() const { return ""; }

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

        reset(currentStreamState, record);
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

    if (!currentStreamState.initialized) {
      if (enoughDataReceived(currentStreamState)) {
        // streamState.initialized = true;
        process(currentStreamState, record, *data);
        // NOTE: To allow derived classes to notice modification of the variable
        // streamState.initialized, it is necessary to set this after calling
        // process.
        currentStreamState.initialized = true;
      }
    } else {
      // Call process to cause a derived processor to work on the data.
      process(currentStreamState, record, *data);
    }

    currentStreamState.lastRecord = record;

  } catch (...) {
    return false;
  }
  return true;
}

void WaveformProcessor::reset(StreamState &streamState, const Record *record) {
  streamState.lastRecord.reset();
}

bool WaveformProcessor::handleGap(StreamState &streamState,
                                  const Record *record, DoubleArrayPtr &data) {
  return true;
}

void WaveformProcessor::fill(StreamState &streamState, const Record *record,
                             DoubleArrayPtr &data) {
  const auto n{static_cast<size_t>(data->size())};
  streamState.receivedSamples += n;

  if (streamState.filter) {
    auto samples{data->typedData()};
    streamState.filter->apply(n, samples);
  }
}

bool WaveformProcessor::enoughDataReceived(
    const StreamState &streamState) const {
  return streamState.receivedSamples > streamState.neededSamples;
}

void WaveformProcessor::emitResult(const Record *record,
                                   const ResultCPtr &result) {
  if (enabled() && _resultCallback) _resultCallback(this, record, result);
}

void WaveformProcessor::setupStream(StreamState &streamState,
                                    const Record *record) {
  const auto &f{record->samplingFrequency()};
  streamState.samplingFrequency = f;
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

void WaveformProcessor::setDebugInfoDir(const boost::filesystem::path &path) {
  _debugInfoDir = path;
}

}  // namespace detect
}  // namespace Seiscomp
