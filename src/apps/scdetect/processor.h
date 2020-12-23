#ifndef SCDETECT_APPS_SCDETECT_PROCESSOR_H_
#define SCDETECT_APPS_SCDETECT_PROCESSOR_H_

#include <memory>
#include <unordered_map>

#include <boost/filesystem.hpp>
#include <boost/function.hpp>

#include <seiscomp/core/baseobject.h>
#include <seiscomp/core/datetime.h>
#include <seiscomp/core/record.h>
#include <seiscomp/core/recordsequence.h>
#include <seiscomp/core/timewindow.h>
#include <seiscomp/math/filter.h>
#include <seiscomp/processing/stream.h>

#include "log.h"

#define SCDETECT_DEBUG_PROCESSOR(processor_ptr, format, ...)                   \
  SCDETECT_DEBUG_TAGGED(processor_ptr->id(), format, ##__VA_ARGS__)
#define SCDETECT_INFO_PROCESSOR(processor_ptr, format, ...)                    \
  SCDETECT_INFO_TAGGED(processor_ptr->id(), format, ##__VA_ARGS__)
#define SCDETECT_WARNING_PROCESSOR(processor_ptr, format, ...)                 \
  SCDETECT_WARNING_TAGGED(processor_ptr->id(), format, ##__VA_ARGS__)
#define SCDETECT_ERROR_PROCESSOR(processor_ptr, format, ...)                   \
  SCDETECT_ERROR_TAGGED(processor_ptr->id(), format, ##__VA_ARGS__)
#define SCDETECT_NOTICE_PROCESSOR(processor_ptr, format, ...)                  \
  SCDETECT_NOTICE_TAGGED(processor_ptr->id(), format, ##__VA_ARGS__)
#define SCDETECT_LOG_PROCESSOR(channel, processor_ptr, format, ...)            \
  SCDETECT_LOG_TAGGED(channel, processor_ptr->id(), format, ##__VA_ARGS__)

namespace Seiscomp {
namespace detect {

DEFINE_SMARTPOINTER(Processor);
class Processor : public Core::BaseObject {

public:
  using Filter = Math::Filtering::InPlaceFilter<double>;

  Processor(const std::string &id,
            const boost::filesystem::path &debug_info_dir = "");

  virtual ~Processor() {}

  DEFINE_SMARTPOINTER(Result);
  class Result : public Core::BaseObject {
  public:
    virtual ~Result();
    /* virtual void Publish() = 0; */
  };

  using PublishResultCallback = boost::function<void(
      ProcessorCPtr processor, RecordCPtr record, ResultCPtr result)>;

  // XXX(damb): From libs/seiscomp/processing/waveformprocessor.h
  enum class Status {
    kWaitingForData = 0,
    // Associated value is progress in [1,99]
    kInProgress,
    kFinished = 100,
    // Associated value is the last status
    kTerminated,
    // Associated value is the failed snr value
    kLowSNR,
    // No associated value yet
    kQCError,
    // Data is clipped
    kDataClipped,
    // Error during deconvolution
    kDeconvolutionFailed,
    // Distance hint is out of range to continue processing
    kDistanceOutOfRange,
    // Depth hint is out of range to continue processing
    kDepthOutOfRange,
    // Unit is not supported, e.g. m/s <> m/s**2
    kIncompatibleUnit,
    // Orientation missing
    kMissingOrientation,
    // Gain missing
    kMissingGain,
    // Response missing
    kMissingResponse,
    // Sampling frequency does not match. Either records of one trace have
    // different sampling frequencies or the sampling frequencies of different
    // channels do not match.
    kInvalidSamplingFreq,
    // No associated value yet (error code?)
    kError,
    // -- The following enumerations were added with API 12 ---
    // No distance hint set
    kMissingDistance,
    //  No depth hint set
    kMissingDepth,
    //  No time hint set
    kMissingTime,
    // No hypocenter (Origin) given
    kMissingHypocenter,
    // No receiver (SensorLocation) given
    kMissingReceiver,
    // No pick (Pick) given
    kMissingPick,
    // Metadata is incomplete, e.g. a particualr stream attribute is not set or
    // empty
    kIncompleteMetadata,
    // The epicentre is out of supported regions
    kEpicenterOutOfRegions,
    // The receiver is out of supported regions
    kReceiverOutOfRegions,
    // The entire raypath does not lie entirely in the supported regions
    kRayPathOutOfRegions,
    // Travel time table lookup failed
    kTravelTimeEstimateFailed,
    // ----
    // Processor cannot handle/process record with streamID
    kInvalidStream
  };

  void enable();
  void disable();
  bool enabled() const;

  // Returns the processor's identifier
  const std::string &id() const;

  virtual void set_result_callback(const PublishResultCallback &callback);

  // Returns the current status of the processor
  Status status() const;

  // Returns the value associated with the status
  double status_value() const;

  // Sets the filter to apply
  virtual void set_filter(Filter *filter) = 0;
  // Returns the processor's initialization time in seconds
  virtual const Core::TimeSpan init_time() const;

  // Sets the maximal gap length in seconds for that missing samples are
  // handled or tolerated. Default: no tolerance
  virtual void set_gap_tolerance(const Core::TimeSpan &duration);
  virtual const Core::TimeSpan &gap_tolerance() const;
  // Enables/disables the linear interpolation of missing samples
  // inside a set gap tolerance
  virtual void set_gap_interpolation(bool e);
  virtual bool gap_interpolation() const;

  // Default implementation returns if the status if greater than
  // Status::kInProgress.
  virtual bool finished() const;

  // Returns the time window processed and correlated
  const Core::TimeWindow &processed() const;

  // Returns the file system path to the processor's debug info directory
  const boost::filesystem::path &debug_info_dir() const;

  // Returns if the processor is operated in debug mode
  bool debug_mode() const;

  // Feed data to the processor (implies a call to the Process() method).
  virtual bool Feed(const Record *record) = 0;

  // Resets the processor completely. The configured init time is going to be
  // processed again.
  virtual void Reset();

  // Terminates the processor ignoring its current status
  void Terminate();

  // Closes the processor meaning that no more records are going to be fed in.
  // The processing has been finished.
  virtual void Close() const;

  // Returns if picks should be created and associated w.r.t. the processing
  // result
  virtual bool WithPicks() const;

  // Returns a debug string for the corresponding processor
  virtual std::string DebugString() const;

protected:
  struct StreamState {
    // Value of the last sample
    double last_sample;

    // Number of samples required to finish initialization
    size_t needed_samples;
    // Number of samples already received
    size_t received_samples;
    // Initialization state
    bool initialized;

    // The last received record of the stream
    RecordCPtr last_record;
    // The complete pre-processed data time window so far
    Core::TimeWindow data_time_window;

    // The sampling frequency of the stream
    double sampling_frequency;
    // The filter (if used)
    Filter *filter{nullptr};
  };

  // Virtual method that must be used in derived classes to analyse a
  // datastream. Both the raw record and the filtered data array is passed.
  virtual void Process(StreamState &stream_state, RecordCPtr record,
                       const DoubleArray &filtered_data) = 0;
  // Store the record
  virtual bool Store(StreamState &stream_state, RecordCPtr record);

  // Handles gaps. Returns whether the gap has been handled or not.
  virtual bool HandleGap(StreamState &stream_state, RecordCPtr record,
                         DoubleArrayPtr data);
  // Fill gaps
  virtual bool FillGap(StreamState &stream_state, RecordCPtr record,
                       const Core::TimeSpan &duration, double next_sample,
                       size_t missing_samples);
  // Fill data and perform filtering (if required)
  virtual void Fill(StreamState &stream_state, RecordCPtr record, size_t n,
                    double *samples);

  // Initially check if the Processor received enough data in order to execute
  // the `Process` method.
  virtual bool EnoughDataReceived(const StreamState &stream_state) const;

  virtual void EmitResult(RecordCPtr record, ResultCPtr result);
  // Initialize the filter for the given sampling frequency
  virtual void InitFilter(StreamState &stream_state, double sampling_freq);

  void set_status(Status status, double value);

  void set_debug_info_dir(const boost::filesystem::path &path);

  void set_processed(const Core::TimeWindow &tw);
  // Merges `tw` with the time window already processed
  void merge_processed(const Core::TimeWindow &tw);

  // Enables saturation check of absolute values of incoming samples and sets
  // the status to DataClipped if checked positive. The data is checked in
  // the Fill method. If derived classes reimplement this method without
  // calling this implementation, the check is not performed.
  void set_saturation_check(bool e);
  // Returns whether saturation check is enabled
  bool saturation_check() const;
  // Sets the saturation threshold. The default is -1
  void set_saturation_threshold(double t);

  bool enabled_{true};

  bool saturation_check_{false};
  double saturation_threshold_{-1};

  // Processor initialization time
  Core::TimeSpan init_time_;

  //! Threshold to recognize a gap
  Core::TimeSpan gap_threshold_;
  // Gap length to tolerate
  Core::TimeSpan gap_tolerance_;

  bool gap_interpolation_{false};

  PublishResultCallback result_callback_;

private:
  std::string id_;

  Status status_{Status::kWaitingForData};
  double status_value_{0};

  Core::TimeWindow processed_{};

  boost::filesystem::path debug_info_dir_;
};

namespace utils {
bool IsValidFilter(const std::string &filter_string, std::string &err);
}

} // namespace detect
} // namespace Seiscomp

#endif // SCDETECT_APPS_SCDETECT_PROCESSOR_H_
