#include "reducing_amplitude_processor.h"

#include <seiscomp/system/environment.h>

#include <algorithm>
#include <boost/filesystem.hpp>
#include <stdexcept>

#include "settings.h"
#include "util/memory.h"
#include "util/util.h"
#include "util/waveform_stream_id.h"
#include "waveform.h"

namespace Seiscomp {
namespace detect {

void ReducingAmplitudeProcessor::setFilter(std::unique_ptr<Filter> &&filter,
                                           const Core::TimeSpan &initTime) {
  if (locked()) {
    return;
  }

  _filter = std::move(filter);
  _initTime = initTime;

  setTimeWindow(timeWindow());
}

bool ReducingAmplitudeProcessor::feed(const Record *record) {
  if (_commonSamplingFrequency &&
      *_commonSamplingFrequency != record->samplingFrequency()) {
    // TODO(damb):
    //
    // - implement resampling; currently we assume a common sampling frequency
    // for all streams
    setStatus(Status::kInvalidSamplingFreq, record->samplingFrequency());
    return false;
  }

  return WaveformProcessor::feed(record);
}

void ReducingAmplitudeProcessor::reset() {
  for (auto &streamPair : _streams) {
    auto &stream{streamPair.second};
    WaveformProcessor::reset(stream.streamState);
    stream.buffer.clear();
  }

  _commonSamplingFrequency = boost::none;
  WaveformProcessor::reset();
}

void ReducingAmplitudeProcessor::add(
    const std::string &netCode, const std::string &staCode,
    const std::string &locCode, const Processing::Stream &streamConfig,
    const AmplitudeProcessor::DeconvolutionConfig &deconvolutionConfig) {
  if (locked()) {
    return;
  }

  StreamItem stream;
  stream.streamConfig = streamConfig;
  stream.deconvolutionConfig = deconvolutionConfig;

  const util::WaveformStreamID waveformStreamId{netCode, staCode, locCode,
                                                streamConfig.code()};

  _streams.emplace(util::to_string(waveformStreamId), std::move(stream));
}

std::vector<std::string> ReducingAmplitudeProcessor::waveformStreamIds() const {
  return util::map_keys(_streams);
}

void ReducingAmplitudeProcessor::dumpBufferedData(std::ostream &out) {
  for (const auto &streamPair : _streams) {
    const auto &buffer{streamPair.second.buffer};
    if (!static_cast<bool>(buffer.size())) {
      continue;
    }

    const util::WaveformStreamID waveformStreamId{streamPair.first};
    GenericRecord trace{waveformStreamId.netCode(), waveformStreamId.staCode(),
                        waveformStreamId.locCode(), waveformStreamId.chaCode(),
                        /*stime=*/safetyTimeWindow().startTime() + _initTime,
                        /*fsamp=*/
                        _commonSamplingFrequency.value_or(
                            streamPair.second.streamState.samplingFrequency)};
    trace.setData(dynamic_cast<DoubleArray *>(buffer.copy(Array::DOUBLE)));

    waveform::write(trace, out);
  }
}

boost::optional<double> ReducingAmplitudeProcessor::reduceNoiseData(
    const std::vector<DoubleArray const *> &data,
    const std::vector<IndexRange> &idxRanges,
    const std::vector<NoiseInfo> &noiseInfos) {
  return boost::none;
}

processing::WaveformProcessor::StreamState *
ReducingAmplitudeProcessor::streamState(const Record *record) {
  return &_streams.at(record->streamID()).streamState;
}

void ReducingAmplitudeProcessor::process(StreamState &streamState,
                                         const Record *record,
                                         const DoubleArray &filteredData) {
  // TODO(damb):
  //
  // - compute noise from reduced data;

  setStatus(Status::kInProgress, 1);

#ifdef SCDETECT_DEBUG
  const auto *env{Seiscomp::Environment::Instance()};
  boost::filesystem::path scInstallDir{env->installDir()};
  boost::filesystem::path pathTemp{scInstallDir / settings::kPathTemp / id()};

  if (util::createDirectory(pathTemp)) {
    // dump buffered filtered (raw) data
    const auto p{pathTemp / "filtered.mseed"};
    std::ofstream ofs{p.string()};
    dumpBufferedData(ofs);
    ofs.close();
  }
#endif

  // buffers are already aligned regarding starttime
  const auto bufferBeginTime{
      _streams.cbegin()->second.streamState.dataTimeWindow.startTime() +
      _initTime};

  const auto itEarliestEndTime{std::min_element(
      _streams.cbegin(), _streams.cend(),
      [](const StreamMap::value_type &lhs, const StreamMap::value_type &rhs) {
        return lhs.second.streamState.dataTimeWindow.endTime() <
               rhs.second.streamState.dataTimeWindow.endTime();
      })};
  const auto bufferEndTime{
      itEarliestEndTime->second.streamState.dataTimeWindow.endTime()};

  const double commonSamplingFrequency{*_commonSamplingFrequency};
  // compute signal offsets
  //
  // - XXX(damb): note that signalBegin() includes the processor's (leading)
  // initialization time
  Core::Time signalStartTime{bufferBeginTime};
  size_t signalBeginIdx{0};
  if (bufferBeginTime < signalBegin() + _initTime) {
    signalBeginIdx =
        static_cast<size_t>((signalBegin() + _initTime - bufferBeginTime) *
                            commonSamplingFrequency);
    signalStartTime = signalBegin() + _initTime;
  }

  if (signalEnd() < bufferBeginTime) {
    setSignalEnd(bufferBeginTime);
  }
  const auto computeSignalEndIdx = [&commonSamplingFrequency, &bufferBeginTime](
                                       const Core::Time &signalEnd) {
    return static_cast<size_t>(
        static_cast<double>(signalEnd - bufferBeginTime) *
            commonSamplingFrequency -
        0.5);
  };
  Core::Time signalEndTime;
  size_t signalEndIdx;
  if (signalEnd() < bufferEndTime) {
    signalEndIdx = computeSignalEndIdx(signalEnd());
    signalEndTime = signalEnd();
  } else {
    signalEndIdx = computeSignalEndIdx(bufferEndTime);
    signalEndTime = bufferEndTime;
  }

  if (signalBeginIdx == signalEndIdx) {
    setStatus(Status::kError, 0);
    return;
  }

  std::vector<DoubleArray const *> data;
  for (auto &streamPair : _streams) {
    auto &stream{streamPair.second};

    // slice buffered data
    stream.buffer.setData(signalEndIdx - signalBeginIdx,
                          stream.buffer.typedData() + signalBeginIdx);

    preprocessData(stream.streamState, stream.streamConfig,
                   stream.deconvolutionConfig, stream.buffer);
    if (finished()) {
      return;
    }

    data.push_back(&stream.buffer);
  }

#ifdef SCDETECT_DEBUG
  if (util::createDirectory(pathTemp)) {
    // dump buffered preprocessed data
    const auto p{pathTemp / "preprocessed.mseed"};
    std::ofstream ofs{p.string()};
    dumpBufferedData(ofs);
    ofs.close();
  }
#endif

  // TODO(damb):
  //
  // - pass noise infos
  std::vector<NoiseInfo> noiseInfos{data.size()};
  auto reduced{reduceAmplitudeData(data, noiseInfos,
                                   IndexRange{signalBeginIdx, signalEndIdx})};
  if (!reduced || reduced->size() <= 0) {
    setStatus(Status::kError, 0);
    return;
  }

#ifdef SCDETECT_DEBUG
  if (_commonSamplingFrequency) {
    if (util::createDirectory(pathTemp)) {
      GenericRecord trace{
          "N", "S", "L", "C", signalStartTime, *_commonSamplingFrequency};
      trace.setData(dynamic_cast<DoubleArray *>(reduced->copy(Array::DOUBLE)));

      const auto p{pathTemp / "reduced.mseed"};
      std::ofstream ofs{p.string()};
      waveform::write(trace, ofs);
      ofs.close();
    }
  }
#endif

  auto amplitude{util::make_smart<Amplitude>()};
  computeAmplitude(*reduced,
                   IndexRange{0, static_cast<size_t>(reduced->size()) - 1},
                   *amplitude);
  if (finished()) {
    return;
  }

  // time window based amplitude time
  amplitude->time.reference = signalStartTime;
  amplitude->time.end = static_cast<double>(signalEndTime - signalStartTime);

  setStatus(Status::kFinished, 100);
  emitAmplitude(record, amplitude);
}

bool ReducingAmplitudeProcessor::store(const Record *record) {
  bool isFirstStreamRecord{false};
  auto *currentStreamState{streamState(record)};
  assert(currentStreamState);

  // check if stream is known
  try {
    isFirstStreamRecord = !static_cast<bool>(currentStreamState->lastRecord);
  } catch (std::out_of_range &) {
    return false;
  }

  // make sure the record does fullfil the `TimeWindowProcessor`'s window
  // requirements
  if (!record->timeWindow().overlaps(safetyTimeWindow()) ||
      (isFirstStreamRecord &&
       (record->timeWindow().startTime() > safetyTimeWindow().startTime()))) {
    return false;
  }

  if (isFirstStreamRecord) {
    // initialize filter
    if (_filter) {
      currentStreamState->filter.reset(_filter->clone());
    }

    auto differenceGreaterEqualSampleTolerance{
        (Core::TimeSpan{record->timeWindow().startTime() -
                        safetyTimeWindow().startTime()}
             .abs() >= Core::TimeSpan{1.0 / record->samplingFrequency()})};
    // trim the first incoming stream records equally at the front to a common
    // start time
    if (record->timeWindow().startTime() < safetyTimeWindow().startTime() &&
        differenceGreaterEqualSampleTolerance) {
      auto firstRecord{util::make_smart<GenericRecord>(*record)};
      // the caller is required to copy the data
      // https://github.com/SeisComP/common/issues/38
      firstRecord->setData(
          dynamic_cast<DoubleArray *>(record->data()->copy(Array::DOUBLE)));

      waveform::trim(
          *firstRecord,
          Core::TimeWindow{safetyTimeWindow().startTime(), record->endTime()});

      return processing::TimeWindowProcessor::store(firstRecord.get());
    }
  }

  return processing::TimeWindowProcessor::store(record);
}

bool ReducingAmplitudeProcessor::fill(processing::StreamState &streamState,
                                      const Record *record,
                                      DoubleArrayPtr &data) {
  auto retval{WaveformProcessor::fill(streamState, record, data)};
  if (retval) {
    const auto &s{dynamic_cast<WaveformProcessor::StreamState &>(streamState)};
    // only buffer samples once the filter is initialized
    if (s.receivedSamples >= s.neededSamples) {
      size_t offset{0};
      if (!s.initialized) {
        // check if processing start lies within the record
        offset = std::max(
            0, static_cast<int>(data->size()) -
                   static_cast<int>(s.receivedSamples - s.neededSamples));
      }

      _streams.at(record->streamID())
          .buffer.append(data->size() - offset, data->typedData() + offset);
    }
  }
  return retval;
}

bool ReducingAmplitudeProcessor::processIfEnoughDataReceived(
    StreamState &streamState, const Record *record,
    const DoubleArray &filteredData) {
  if (!_commonSamplingFrequency) {
    _commonSamplingFrequency = streamState.samplingFrequency;
  }

  bool processed{false};
  if (enoughDataReceived(streamState)) {
    process(streamState, record, filteredData);
    processed = true;
  }

  if (!streamState.initialized &&
      (streamState.receivedSamples >= streamState.neededSamples)) {
    streamState.initialized = true;
  }
  return processed;
}

bool ReducingAmplitudeProcessor::enoughDataReceived(
    const StreamState &streamState) const {
  if (!_commonSamplingFrequency) {
    return false;
  }

  auto neededSamples{static_cast<size_t>(
      safetyTimeWindow().length() * *_commonSamplingFrequency + 0.5)};
  return std::all_of(_streams.cbegin(), _streams.cend(),
                     [&neededSamples](const StreamMap::value_type &streamPair) {
                       return streamPair.second.streamState.receivedSamples >=
                              neededSamples;
                     });
}

bool ReducingAmplitudeProcessor::locked() const {
  return static_cast<bool>(_commonSamplingFrequency);
}

}  // namespace detect
}  // namespace Seiscomp
