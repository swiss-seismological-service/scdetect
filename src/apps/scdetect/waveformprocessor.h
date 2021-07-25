#ifndef SCDETECT_APPS_SCDETECT_WAVEFORMPROCESSOR_H_
#define SCDETECT_APPS_SCDETECT_WAVEFORMPROCESSOR_H_

#include <seiscomp/core/baseobject.h>
#include <seiscomp/core/datetime.h>
#include <seiscomp/core/record.h>
#include <seiscomp/core/recordsequence.h>
#include <seiscomp/core/timewindow.h>
#include <seiscomp/math/filter.h>

#include <functional>
#include <memory>

#include "processor.h"

namespace Seiscomp {
namespace detect {

class WaveformOperator;

// Abstract interface for waveform processors
class WaveformProcessor : public Processor {
 public:
  using Filter = Math::Filtering::InPlaceFilter<double>;

  WaveformProcessor(const std::string &id);

  DEFINE_SMARTPOINTER(Result);
  class Result : public Core::BaseObject {
   public:
    virtual ~Result();
    /* virtual void Publish() = 0; */
  };

  using PublishResultCallback = std::function<void(
      const WaveformProcessor *, const Record *, const ResultCPtr &)>;

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

  // Sets the result callback in order to publish processing results
  void setResultCallback(const PublishResultCallback &callback);

  // Returns the current status of the processor
  Status status() const;

  // Returns the value associated with the status
  double statusValue() const;

  // Sets the filter to apply; the filter pointer passed is owned by the
  // `WaveformProcessor`
  virtual void setFilter(Filter *filter, const Core::TimeSpan &initTime) = 0;
  // Configures a `WaveformProcessor` with `op`. `op` is applied to all records
  // fed. `op` sits between `feed` and `store`. The pointer ownership goes to
  // the processor.
  void setOperator(WaveformOperator *op);
  // Returns the processor's initialization time
  virtual const Core::TimeSpan initTime() const;

  // Default implementation returns if the status if greater than
  // Status::kInProgress.
  virtual bool finished() const;

  // Returns the time window processed and correlated
  virtual const Core::TimeWindow &processed() const = 0;

  // Returns if the processor is operated in debug mode
  bool debugMode() const;
  // Enables/disables the processor's debug mode
  void setDebugMode(bool debugMode);

  // Feed data to the processor (implies a call to the process() method).
  virtual bool feed(const Record *record);

  // Resets the processor completely. The configured init time is going to be
  // processed again.
  virtual void reset();

  // Terminates the processor ignoring its current status
  virtual void terminate();

  // Closes the processor meaning that no more records are going to be fed in.
  // The processing has been finished.
  virtual void close() const;

  // Returns a debug string for the corresponding processor
  virtual std::string debugString() const;

 protected:
  // Describes the current state of a stream
  struct StreamState {
    ~StreamState();
    // Value of the last sample
    double lastSample{0};

    // Number of samples required to finish initialization
    size_t neededSamples{0};
    // Number of samples already received
    size_t receivedSamples{0};
    // Initialization state
    bool initialized{false};

    // The last received record of the stream
    RecordCPtr lastRecord;
    // The complete processed data time window so far
    Core::TimeWindow dataTimeWindow;

    // The sampling frequency of the stream
    double samplingFrequency{0};
    // The filter (if used)
    Filter *filter{nullptr};
  };

  virtual StreamState &streamState(const Record *record) = 0;

  // Virtual method that must be used in derived classes to analyse a
  // data stream. Both the raw record and the filtered data array is passed.
  virtual void process(StreamState &streamState, const Record *record,
                       const DoubleArray &filteredData) = 0;
  // Store the record
  virtual bool store(const Record *record);

  // Resets the `WaveformProcessor` with regards to `streamState` and
  // `record`.
  virtual void reset(StreamState &streamState, const Record *record);

  // Handles gaps. Returns whether the gap has been handled or not.
  virtual bool handleGap(StreamState &streamState, const Record *record,
                         DoubleArrayPtr &data);

  // Fill data and perform filtering (if required)
  virtual void fill(StreamState &streamState, const Record *record,
                    DoubleArrayPtr &data);

  // Initially check if the `WaveformProcessor` received enough data in order to
  // execute the `process` method.
  virtual bool enoughDataReceived(const StreamState &streamState) const;

  virtual void emitResult(const Record *record, const ResultCPtr &result);
  // Setup and initialize the stream
  virtual void setupStream(StreamState &streamState, const Record *record);

  void setStatus(Status status, double value);

  bool _enabled{true};

  // WaveformProcessor initialization time
  Core::TimeSpan _initTime;

  PublishResultCallback _resultCallback;

  std::unique_ptr<WaveformOperator> _waveformOperator;

 private:
  Status _status{Status::kWaitingForData};
  double _statusValue{0};

  bool _debugMode{false};
};

}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_WAVEFORMPROCESSOR_H_
