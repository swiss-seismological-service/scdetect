#include "amplitudeprocessor.h"

#include <seiscomp/core/genericrecord.h>
#include <seiscomp/math/filter/iirdifferentiate.h>
#include <seiscomp/math/mean.h>

#include <algorithm>
#include <cstddef>
#include <stdexcept>

#include "settings.h"
#include "utils.h"
#include "waveform.h"
#include "waveformoperator.h"

namespace Seiscomp {
namespace detect {

AmplitudeProcessor::AmplitudeProcessor(const std::string &id)
    : TimeWindowProcessor{id} {}

void AmplitudeProcessor::setSignalBegin(
    const boost::optional<Core::TimeSpan> &signalBegin) {
  if (!signalBegin) {
    _config.signalBegin = signalBegin;
    return;
  }

  if (*signalBegin > Core::TimeSpan{0.0} &&
      safetyTimeWindow().endTime() - _config.signalEnd.value_or(0.0) >
          safetyTimeWindow().startTime() + *signalBegin) {
    _config.signalBegin = signalBegin;
  }
}

Core::Time AmplitudeProcessor::signalBegin() const {
  return safetyTimeWindow().startTime() + _config.signalBegin.value_or(0.0);
}

void AmplitudeProcessor::setSignalEnd(
    const boost::optional<Core::TimeSpan> &signalEnd) {
  if (!signalEnd) {
    _config.signalEnd = signalEnd;
    return;
  }

  if (*signalEnd > Core::TimeSpan{0.0} &&
      safetyTimeWindow().endTime() - *signalEnd >
          safetyTimeWindow().startTime() + _config.signalBegin.value_or(0.0)) {
    _config.signalEnd = signalEnd;
  }
}

Core::Time AmplitudeProcessor::signalEnd() const {
  return safetyTimeWindow().endTime() - _config.signalEnd.value_or(0.0);
}

const std::string &AmplitudeProcessor::type() const { return _type; }

const std::string &AmplitudeProcessor::unit() const { return _unit; }

void AmplitudeProcessor::setEnvironment(
    const DataModel::OriginCPtr &hypocenter,
    const DataModel::SensorLocationCPtr &receiver,
    const std::vector<DataModel::PickCPtr> &picks) {
  _environment.hypocenter = hypocenter;
  _environment.receiver = receiver;
  _environment.picks = picks;
}

const AmplitudeProcessor::Environment &AmplitudeProcessor::environment() const {
  return _environment;
}

void AmplitudeProcessor::finalize(DataModel::Amplitude *amplitude) const {}

void AmplitudeProcessor::preprocessData(
    StreamState &streamState, Processing::Sensor *sensor,
    const DeconvolutionConfig &deconvolutionConfig, DoubleArray &data) {}

bool AmplitudeProcessor::computeNoise(const DoubleArray &data,
                                      const IndexRange &idxRange,
                                      NoiseInfo &noiseInfo) {
  // compute offset and rms within the time window
  size_t beginIdx{idxRange.begin}, endIdx{idxRange.end};
  if (beginIdx < 0) beginIdx = 0;
  if (endIdx < 0) return false;
  if (endIdx > static_cast<size_t>(data.size()))
    endIdx = static_cast<size_t>(data.size());

  // If noise window is zero return an amplitude and offset of zero as well.
  if (endIdx - beginIdx == 0) {
    noiseInfo.offset = 0;
    noiseInfo.amplitude = 0;
    return true;
  }

  DoubleArrayPtr sliced{
      static_cast<DoubleArray *>(data.slice(beginIdx, endIdx))};
  if (!sliced) {
    return false;
  }

  // compute noise offset as the median
  double offset{sliced->median()};
  // compute rms while removing offset
  double amplitude{2 * sliced->rms(offset)};

  if (offset) noiseInfo.offset = offset;
  if (amplitude) noiseInfo.amplitude = amplitude;

  return true;
}

bool AmplitudeProcessor::deconvolveData(StreamState &streamState,
                                        Processing::Response *resp,
                                        const DeconvolutionConfig &config,
                                        int numberOfIntegrations,
                                        DoubleArray &data) {
  double m, n;
  // remove linear trend
  Math::Statistics::computeLinearTrend(data.size(), data.typedData(), m, n);
  Math::Statistics::detrend(data.size(), data.typedData(), m, n);

  // XXX(damb): integration is implemented by means of deconvolution i.e. by
  // means of adding an additional zero to the nominator of the rational
  // transfer function
  if (!resp->deconvolveFFT(
          data, streamState.samplingFrequency, config.responseTaperLength,
          config.minimumResponseTaperFrequency,
          config.maximumResponseTaperFrequency,
          numberOfIntegrations < 0 ? 0 : numberOfIntegrations)) {
    return false;
  }

  if (numberOfIntegrations < 0) {
    if (!deriveData(streamState, abs(numberOfIntegrations), data)) {
      return false;
    }
  }

  return true;
}

bool AmplitudeProcessor::deriveData(StreamState &streamState,
                                    int numberOfDerivations,
                                    DoubleArray &data) {
  while (numberOfDerivations > 0) {
    Math::Filtering::IIRDifferentiate<double> diff;
    diff.setSamplingFrequency(streamState.samplingFrequency);
    diff.apply(data.size(), data.typedData());
    --numberOfDerivations;
  }

  return true;
}

/* ------------------------------------------------------------------------- */
ReducingAmplitudeProcessor::ReducingAmplitudeProcessor(const std::string &id)
    : AmplitudeProcessor{id} {}

void ReducingAmplitudeProcessor::setFilter(Filter *filter,
                                           const Core::TimeSpan &initTime) {
  if (!locked()) {
    _filter.reset(filter);
    _initTime = initTime;

    setTimeWindow(timeWindow());
  }
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

  if (WaveformProcessor::feed(record)) {
    _commonSamplingFrequency = record->samplingFrequency();
  }
  return false;
}

void ReducingAmplitudeProcessor::reset() {
  for (auto &streamPair : _streams) {
    auto &stream{streamPair.second};
    WaveformProcessor::reset(stream.streamState);
    stream.buffer.clear();
  }

  _commonSamplingFrequency = boost::none;
}

void ReducingAmplitudeProcessor::add(const std::string &netCode,
                                     const std::string &staCode,
                                     const std::string &locCode,
                                     const Processing::Stream &streamConfig) {
  if (!locked()) {
    DeconvolutionConfig deconvolutionConfig;
    deconvolutionConfig.enabled = true;

    StreamItem stream;
    stream.streamConfig = streamConfig;
    stream.deconvolutionConfig = deconvolutionConfig;

    const utils::WaveformStreamID waveformStreamId{netCode, staCode, locCode,
                                                   streamConfig.code()};

    _streams.emplace(utils::to_string(waveformStreamId), stream);
  }
}

std::vector<std::string> ReducingAmplitudeProcessor::waveformStreamIds() const {
  return utils::map_keys(_streams);
}

boost::optional<double> ReducingAmplitudeProcessor::reduceNoiseData(
    const std::vector<DoubleArray const *> &data,
    const std::vector<IndexRange> &idxRanges,
    const std::vector<NoiseInfo> &noiseInfos) {
  return boost::none;
}

WaveformProcessor::StreamState &ReducingAmplitudeProcessor::streamState(
    const Record *record) {
  return _streams.at(record->streamID()).streamState;
}

void ReducingAmplitudeProcessor::process(StreamState &streamState,
                                         const Record *record,
                                         const DoubleArray &filteredData) {
  // TODO(damb):
  //
  // - call compute()
  // - compute noise from reduced data;

  setStatus(Status::kInProgress, 1);

  std::vector<DoubleArray const *> data;
  for (auto &streamPair : _streams) {
    auto &stream{streamPair.second};
    preprocessData(stream.streamState, stream.streamConfig.sensor(),
                   stream.deconvolutionConfig, stream.buffer);
    if (finished()) {
      return;
    }
    data.push_back(&stream.buffer);
  }

  // buffers are already aligned regarding starttime
  const auto bufferBeginTime{
      _streams.cbegin()->second.streamState.dataTimeWindow.startTime()};

  const auto itEarliestEndTime{std::min_element(
      _streams.cbegin(), _streams.cend(),
      [](const StreamMap::value_type &lhs, const StreamMap::value_type &rhs) {
        return lhs.second.streamState.dataTimeWindow.endTime() <
               rhs.second.streamState.dataTimeWindow.endTime();
      })};
  const auto bufferEndTime{
      itEarliestEndTime->second.streamState.dataTimeWindow.endTime()};

  double commonSamplingFrequency{_commonSamplingFrequency.value_or(0)};
  // compute signal offsets
  Core::Time signalStartTime{bufferBeginTime};
  size_t signalBeginIdx{0};
  if (bufferBeginTime < signalBegin()) {
    signalBeginIdx = static_cast<size_t>((signalBegin() - bufferBeginTime) *
                                         commonSamplingFrequency);
    signalStartTime = signalBegin();
  }
  if (signalEnd() < bufferBeginTime) {
    setSignalEnd(bufferEndTime);
  }
  const auto computeSignalEndIdx =
      [&commonSamplingFrequency, &bufferBeginTime](const Core::Time signalEnd) {
        return static_cast<size_t>(
            static_cast<double>(signalEnd - bufferBeginTime) *
            commonSamplingFrequency);
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

  auto amplitude{utils::make_smart<Amplitude>()};
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
  emitResult(record, amplitude);
}

bool ReducingAmplitudeProcessor::store(const Record *record) {
  bool isFirstStreamRecord{false};
  // check if stream is known
  try {
    isFirstStreamRecord = !static_cast<bool>(streamState(record).lastRecord);
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

  // trim the first incoming stream records equally at the front to a common
  // start time
  if (isFirstStreamRecord &&
      record->timeWindow().startTime() < safetyTimeWindow().startTime()) {
    auto firstRecord{utils::make_unique<GenericRecord>(*record)};
    waveform::trim(
        *firstRecord,
        Core::TimeWindow{safetyTimeWindow().startTime(), record->endTime()});
    return WaveformProcessor::store(firstRecord.release());
  }

  return WaveformProcessor::store(record);
}

bool ReducingAmplitudeProcessor::fill(detect::StreamState &streamState,
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
  return std::all_of(_streams.cbegin(), _streams.cend(),
                     [](const StreamMap::value_type &streamPair) {
                       return streamPair.second.streamState.receivedSamples >=
                              streamPair.second.neededSamples;
                     });
}

void ReducingAmplitudeProcessor::setupStream(StreamState &streamState,
                                             const Record *record) {
  WaveformProcessor::setupStream(streamState, record);

  _streams.at(record->streamID()).neededSamples = static_cast<size_t>(
      safetyTimeWindow().length() * streamState.samplingFrequency + 0.5);
}

bool ReducingAmplitudeProcessor::locked() const {
  return static_cast<bool>(_commonSamplingFrequency);
}

}  // namespace detect
}  // namespace Seiscomp
