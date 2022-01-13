#ifndef SCDETECT_APPS_SCDETECT_OPERATOR_RINGBUFFER_H_
#define SCDETECT_APPS_SCDETECT_OPERATOR_RINGBUFFER_H_

#include <seiscomp/core/datetime.h>
#include <seiscomp/core/recordsequence.h>
#include <seiscomp/core/timewindow.h>
#include <seiscomp/core/typedarray.h>

#include <memory>
#include <string>
#include <unordered_map>

#include "../processing/detail/gap_interpolate.h"
#include "../processing/stream.h"
#include "../processing/waveform_operator.h"
#include "../processing/waveform_processor.h"

namespace Seiscomp {
namespace detect {
namespace waveform_operator {

// `WaveformOperator` implementation providing buffering facilities for `N`
// streams
//
// - implements gap interpolation facilities
class RingBufferOperator : public processing::WaveformOperator,
                           public processing::detail::InterpolateGaps {
 public:
  using WaveformStreamID = std::string;
  using RingBuffer = Seiscomp::RingBuffer;

  RingBufferOperator(processing::WaveformProcessor *waveformProcessor);
  RingBufferOperator(processing::WaveformProcessor *waveformProcessor,
                     Core::TimeSpan bufferSize);
  RingBufferOperator(processing::WaveformProcessor *waveformProcessor,
                     Core::TimeSpan bufferSize,
                     const std::vector<WaveformStreamID> &wfStreamIds);

  processing::WaveformProcessor::Status feed(const Record *record) override;

  void reset() override;

  // Subscribe a stream identified by `wfStreamId` for buffering
  void add(WaveformStreamID wfStreamId);
  // Subscribe a stream identified by `wfStreamId` for buffering while using
  // `bufferSize`
  void add(WaveformStreamID wfStreamId, Core::TimeSpan bufferSize);

  // Returns a shared reference to the buffer identified by `wfStreamId`
  const std::shared_ptr<RingBuffer> &get(WaveformStreamID wfStreamId);

 protected:
  bool store(processing::StreamState &streamState, const Record *record);

  bool fill(processing::StreamState &streamState, const Record *record,
            DoubleArrayPtr &data) override;

  void setupStream(processing::StreamState &streamState, const Record *record);

  void reset(processing::StreamState &streamState);

 private:
  struct StreamConfig {
    processing::StreamState streamState;

    // Reference to the stream buffer
    std::shared_ptr<RingBuffer> streamBuffer;
  };

  using StreamConfigs = std::unordered_map<std::string, StreamConfig>;
  StreamConfigs _streamConfigs;

  Core::TimeSpan _bufferSize{60};

  // Reference to the processor using the operator
  processing::WaveformProcessor *_processor;
};

}  // namespace waveform_operator
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_OPERATOR_RINGBUFFER_H_
