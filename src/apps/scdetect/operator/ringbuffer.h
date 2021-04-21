#ifndef SCDETECT_APPS_SCDETECT_OPERATOR_RINGBUFFER_H_
#define SCDETECT_APPS_SCDETECT_OPERATOR_RINGBUFFER_H_

#include <ios>
#include <memory>
#include <string>
#include <unordered_map>

#include <seiscomp/core/datetime.h>
#include <seiscomp/core/recordsequence.h>
#include <seiscomp/core/timewindow.h>
#include <seiscomp/core/typedarray.h>

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

  RingBufferOperator(WaveformProcessor *waveform_processor);
  RingBufferOperator(WaveformProcessor *waveform_processor,
                     Core::TimeSpan buffer_size);
  RingBufferOperator(WaveformProcessor *waveform_processor,
                     Core::TimeSpan buffer_size,
                     const std::vector<WaveformStreamID> &wf_stream_ids);

  // Enables/disables the linear interpolation of missing samples
  // if the gap is smaller than the configured gap tolerance
  void set_gap_interpolation(bool gap_interpolation);
  // Returns if gap interpolation is enabled or disabled, respectively
  bool gap_interpolation() const;
  // Sets the threshold (i.e. the minimum gap length) for gap interpolation
  void set_gap_threshold(const Core::TimeSpan &duration);
  // Returns the gap threshold configured
  const Core::TimeSpan gap_threshold() const;
  // Sets the maximum gap length to be tolerated
  void set_gap_tolerance(const Core::TimeSpan &duration);
  // Returns the gap tolerance configured
  const Core::TimeSpan gap_tolerance() const;

  WaveformProcessor::Status Feed(const Record *record) override;

  void Reset() override;

  // Subscribe a stream identified by `wf_stream_id` for buffering
  void Add(WaveformStreamID wf_stream_id);
  // Subscribe a stream identified by `wf_stream_id` for buffering while using
  // `buffer_size`
  void Add(WaveformStreamID wf_stream_id, Core::TimeSpan buffer_size);

  // Returns a shared reference to the buffer identified by `wf_stream_id`
  const std::shared_ptr<RingBuffer> &Get(WaveformStreamID wf_stream_id);

protected:
  struct StreamState {
    // Value of the last sample
    double last_sample{0};
    // The last record received
    RecordCPtr last_record;
    // The overall time window received
    Core::TimeWindow data_time_window;
    // The sampling frequency of the stream
    double sampling_frequency{0};
  };

  bool Store(StreamState &stream_state, const Record *record);

  bool HandleGap(StreamState &stream_state, const Record *record,
                 DoubleArrayPtr &data);

  bool Fill(StreamState &stream_state, const Record *record,
            DoubleArrayPtr &data);

  void SetupStream(StreamState &stream_state, const Record *record);

private:
  // Fill gaps
  bool FillGap(StreamState &stream_state, const Record *record,
               const Core::TimeSpan &duration, double next_sample,
               size_t missing_samples);

  struct StreamConfig {
    StreamState stream_state;

    // Reference to the stream buffer
    std::shared_ptr<RingBuffer> stream_buffer;
  };

  using StreamConfigs = std::unordered_map<std::string, StreamConfig>;
  StreamConfigs stream_configs_;

  Core::TimeSpan buffer_size_{30.0 * settings::kBufferMultiplicator};

  // Indicates if gap interpolation is enabled/disabled
  bool gap_interpolation_{false};
  // The minimum gap length to detect a gap
  Core::TimeSpan gap_threshold_;
  // The maximum gap length to tolerate
  Core::TimeSpan gap_tolerance_;

  // Reference to the processor using the operator
  WaveformProcessor *waveform_processor_;
};

} // namespace waveform_operator
} // namespace detect
} // namespace Seiscomp

#endif // SCDETECT_APPS_SCDETECT_OPERATOR_RINGBUFFER_H_
