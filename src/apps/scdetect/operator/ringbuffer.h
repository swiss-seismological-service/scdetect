#ifndef SCDETECT_APPS_SCDETECT_OPERATOR_RINGBUFFER_H_
#define SCDETECT_APPS_SCDETECT_OPERATOR_RINGBUFFER_H_

#include <seiscomp/core/datetime.h>
#include <seiscomp/core/recordsequence.h>
#include <seiscomp/core/timewindow.h>
#include <seiscomp/core/typedarray.h>

#include <memory>
#include <string>
#include <unordered_map>

#include "../settings.h"
#include "../waveformoperator.h"
#include "../waveformprocessor.h"

namespace Seiscomp {
namespace detect {
namespace waveform_operator {

// `WaveformOperator` implementation providing buffering facilities for `N`
// streams
// - implements gap interpolation facilities
class RingBufferOperator : public WaveformOperator {
 public:
  using WaveformStreamID = std::string;
  using RingBuffer = Seiscomp::RingBuffer;

  RingBufferOperator(WaveformProcessor *waveformProcessor);
  RingBufferOperator(WaveformProcessor *waveformProcessor,
                     Core::TimeSpan bufferSize);
  RingBufferOperator(WaveformProcessor *waveformProcessor,
                     Core::TimeSpan bufferSize,
                     const std::vector<WaveformStreamID> &wfStreamIds);

  // Enables/disables the linear interpolation of missing samples
  // if the gap is smaller than the configured gap tolerance
  void setGapInterpolation(bool gapInterpolation);
  // Returns if gap interpolation is enabled or disabled, respectively
  bool gapInterpolation() const;
  // Sets the threshold (i.e. the minimum gap length) for gap interpolation
  void setGapThreshold(const Core::TimeSpan &duration);
  // Returns the gap threshold configured
  const Core::TimeSpan gapThreshold() const;
  // Sets the maximum gap length to be tolerated
  void setGapTolerance(const Core::TimeSpan &duration);
  // Returns the gap tolerance configured
  const Core::TimeSpan gapTolerance() const;

  WaveformProcessor::Status feed(const Record *record) override;

  void reset() override;

  // Subscribe a stream identified by `wfStreamId` for buffering
  void add(WaveformStreamID wfStreamId);
  // Subscribe a stream identified by `wfStreamId` for buffering while using
  // `bufferSize`
  void add(WaveformStreamID wfStreamId, Core::TimeSpan bufferSize);

  // Returns a shared reference to the buffer identified by `wfStreamId`
  const std::shared_ptr<RingBuffer> &get(WaveformStreamID wfStreamId);

 protected:
  struct StreamState {
    // Value of the last sample
    double lastSample{0};
    // The last record received
    RecordCPtr lastRecord;
    // The overall time window received
    Core::TimeWindow dataTimeWindow;
    // The sampling frequency of the stream
    double samplingFrequency{0};
  };

  bool store(StreamState &streamState, const Record *record);

  bool handleGap(StreamState &streamState, const Record *record,
                 DoubleArrayPtr &data);

  bool fill(StreamState &streamState, const Record *record,
            DoubleArrayPtr &data);

  void setupStream(StreamState &streamState, const Record *record);

 private:
  // Fill gaps
  bool fillGap(StreamState &streamState, const Record *record,
               const Core::TimeSpan &duration, double nextSample,
               size_t missingSamples);

  struct StreamConfig {
    StreamState streamState;

    // Reference to the stream buffer
    std::shared_ptr<RingBuffer> streamBuffer;
  };

  using StreamConfigs = std::unordered_map<std::string, StreamConfig>;
  StreamConfigs _streamConfigs;

  Core::TimeSpan _bufferSize{30.0 * settings::kBufferMultiplicator};

  // Indicates if gap interpolation is enabled/disabled
  bool _gapInterpolation{false};
  // The minimum gap length to detect a gap
  Core::TimeSpan _gapThreshold;
  // The maximum gap length to tolerate
  Core::TimeSpan _gapTolerance;

  // Reference to the processor using the operator
  WaveformProcessor *_processor;
};

}  // namespace waveform_operator
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_OPERATOR_RINGBUFFER_H_
