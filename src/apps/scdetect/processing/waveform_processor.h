#ifndef SCDETECT_APPS_SCDETECT_PROCESSING_WAVEFORMPROCESSOR_H_
#define SCDETECT_APPS_SCDETECT_PROCESSING_WAVEFORMPROCESSOR_H_

#include <seiscomp/core/baseobject.h>
#include <seiscomp/core/datetime.h>
#include <seiscomp/core/record.h>
#include <seiscomp/core/timewindow.h>
#include <seiscomp/core/typedarray.h>
#include <seiscomp/math/filter.h>

#include <boost/optional/optional.hpp>
#include <functional>
#include <memory>

#include "mixin/gap_interpolate.h"
#include "processor.h"
#include "stream.h"

namespace Seiscomp {
namespace detect {
namespace processing {

class WaveformOperator;

// Abstract interface for waveform processors.
//
// - implements gap interpolation
//
// - The interface is similar to the one from
// `Seiscomp::Processing::WaveformProcessor`, but it additionally simplifies
// the implementation of *hierarchical* and *composite* waveform processors.
// It is designed in a way that it does neither force implementations to use
// just a single stream nor does it introduce the *concept of a station* (e.g.
// by means of limiting the usage of maximum three channels).
//
class WaveformProcessor : public Processor,
                          public InterpolateGaps<WaveformProcessor> {
 public:
  using Filter = Math::Filtering::InPlaceFilter<double>;

  DEFINE_SMARTPOINTER(Result);
  class Result : public Core::BaseObject {
   public:
    virtual ~Result();
  };

  using PublishResultCallback = std::function<void(
      const WaveformProcessor *, const Record *, const ResultCPtr)>;

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
  };

  // Sets the filter to apply
  //
  // - the `filter` pointer passed is owned by the `WaveformProcessor`
  virtual void setFilter(Filter *filter, const Core::TimeSpan &initTime) = 0;

  void enable();
  void disable();
  bool enabled() const;

  // Sets the result callback in order to publish processing results
  void setResultCallback(const PublishResultCallback &callback);

  // Enables/disables validating whether data is saturated.
  //
  // - If set a saturation check is performed where it is checked whether the
  // data exceeds `threshold`.
  // - The saturation check may be disabled if `boost::none` is passed.
  void setSaturationThreshold(const boost::optional<double> &threshold);

  // Returns the current status of the processor
  Status status() const;

  // Returns the value associated with the status
  double statusValue() const;

  // Configures a `WaveformProcessor` with `op`. `op` is applied to all records
  // fed.
  //
  // - `op` sits between `feed` and `store`
  // - the pointer ownership goes to the processor
  void setOperator(WaveformOperator *op);
  // Returns the processor's initialization time; by default this corresponds
  // to the processor's filter initialization time
  virtual const Core::TimeSpan initTime() const;

  // Default implementation returns if the status if greater than
  // `Status::kInProgress`.
  virtual bool finished() const;

  // Feed data to the processor (implies a call to the process() method).
  virtual bool feed(const Record *record);

  // Resets the processor completely. The configured init time is going to be
  // processed again.
  virtual void reset();

  // Terminates the processor ignoring its current status
  virtual void terminate();

  // Closes the processor meaning that no more records are going to be fed.
  // The processing has been finished.
  virtual void close() const;

 protected:
  // Describes the current state of a stream
  struct StreamState : public processing::StreamState {
    ~StreamState() override;

    // Number of samples required to finish initialization
    size_t neededSamples{0};
    // Number of samples already received
    size_t receivedSamples{0};
    // Initialization state
    bool initialized{false};

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

  // Resets the `WaveformProcessor` with regard to `streamState`
  virtual void reset(StreamState &streamState);

  // Fill data and perform filtering (if required)
  bool fill(processing::StreamState &streamState, const Record *record,
            DoubleArrayPtr &data) override;

  // Check whether data exceeds saturation threshold. The default
  // implementation does not perform any check
  //
  // - returns `true` in case `data` is saturated, else `false`
  virtual bool checkIfSaturated(DoubleArrayPtr &data);

  // Wrapper method for both `enoughDataReceived()` and `process()`. Returns
  // `true` if `process` was called, else `false`
  virtual bool processIfEnoughDataReceived(StreamState &streamState,
                                           const Record *record,
                                           const DoubleArray &filteredData);

  // Initially check if the `WaveformProcessor` received enough data in order to
  // execute the `process()` method.
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

  // Threshold used for the saturation check
  boost::optional<double> _saturationThreshold;

 private:
  Status _status{Status::kWaitingForData};
  double _statusValue{0};
};

std::unique_ptr<WaveformProcessor::Filter> createFilter(
    const std::string &filter);

}  // namespace processing
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_PROCESSING_WAVEFORMPROCESSOR_H_
