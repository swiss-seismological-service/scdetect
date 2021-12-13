#ifndef SCDETECT_APPS_SCDETECT_DETECTOR_DETECTOR_H_
#define SCDETECT_APPS_SCDETECT_DETECTOR_DETECTOR_H_

#include <seiscomp/core/datetime.h>
#include <seiscomp/core/record.h>
#include <seiscomp/core/timewindow.h>
#include <seiscomp/datamodel/origin.h>

#include <boost/filesystem.hpp>
#include <boost/optional.hpp>
#include <cmath>
#include <deque>
#include <functional>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "../exception.h"
#include "../processor.h"
#include "../waveformoperator.h"
#include "../waveformprocessor.h"
#include "arrival.h"
#include "linker.h"
#include "linker/association.h"
#include "linker/strategy.h"
#include "templatewaveformprocessor.h"

namespace Seiscomp {
namespace detect {
namespace detector {

class Detector : public detect::Processor {
 public:
  Detector(const DataModel::OriginCPtr &origin);
  virtual ~Detector();

  class BaseException : public Processor::BaseException {
   public:
    using Processor::BaseException::BaseException;
    BaseException();
  };

  class ProcessingError : public BaseException {
   public:
    using BaseException::BaseException;
    ProcessingError();
  };

  class TemplateMatchingError : public ProcessingError {
   public:
    using ProcessingError::ProcessingError;
    TemplateMatchingError();
  };

  enum class Status { kWaitingForData, kTerminated };

  struct SensorLocation {
    double latitude;
    double longitude;

    // The unique station's identifier (usually the station's public id)
    std::string stationId;
  };

  struct Result {
    Core::Time originTime;
    double fit;

    struct TemplateResult {
      Arrival arrival;
      SensorLocation sensorLocation;

      Core::Time templateWaveformStartTime;
      Core::Time templateWaveformEndTime;
      // The template waveform reference time
      Core::Time templateWaveformReferenceTime;
    };

    using TemplateResults =
        std::unordered_multimap<std::string, TemplateResult>;
    TemplateResults templateResults;

    size_t numChannelsUsed;
    size_t numChannelsAssociated;
    size_t numStationsUsed;
    size_t numStationsAssociated;
  };

  // Returns the detector's status
  Status status() const;
  // Returns the overall time window processed
  const Core::TimeWindow &processed() const;
  // Returns `true` if the detector is currently triggered, else `false`
  bool triggered() const;
  // Enables trigger duration facilities with `duration`
  void enableTrigger(const Core::TimeSpan &duration);
  // Disables trigger duration facilities
  void disableTrigger();
  // Set the trigger thresholds
  void setTriggerThresholds(double triggerOn, double triggerOff = 1);
  // Set the maximum arrival offset threshold
  void setArrivalOffsetThreshold(const boost::optional<double> &thres);
  // Returns the arrival offset threshold configured
  boost::optional<double> arrivalOffsetThreshold() const;
  // Configures the detector with a minimum number of arrivals required to
  // declare an event as a detection
  void setMinArrivals(const boost::optional<size_t> &n);
  // Returns the minimum number of arrivals required in order to declare an
  // event as a detection
  boost::optional<size_t> minArrivals() const;
  // Sets the merging strategy applied while linking
  void setMergingStrategy(linker::MergingStrategy::Type mergingStrategyTypeId);
  // Sets the maximum data latency w.r.t. `NOW`. If configured with
  // `boost::none` latency is not taken into account and thus not validated
  void setMaxLatency(const boost::optional<Core::TimeSpan> &latency);
  // Returns the maximum allowed data latency configured
  boost::optional<Core::TimeSpan> maxLatency() const;
  // Returns the number of registered template processors
  size_t getProcessorCount() const;

  // Register the template waveform processor `proc`. Records are identified by
  // the waveform stream identifier `waveformStreamId`. `proc` is registered
  // together with the template arrival `arrival` and the sensor location
  // `loc`.
  void add(std::unique_ptr<TemplateWaveformProcessor> &&proc,
           const std::string &waveformStreamId, const Arrival &arrival,
           const Detector::SensorLocation &loc,
           const boost::optional<double> &mergingThreshold);
  // Removes the processors processing streams identified by `waveformStreamId`
  void remove(const std::string &waveformStreamId);

  void process(const Record *record);
  // Reset the detector
  void reset();
  // Terminates the detector
  //
  // - if triggered forces flushing the pending detection
  void terminate();

  using PublishResultCallback = std::function<void(const Result &result)>;
  void setResultCallback(const PublishResultCallback &callback);

 protected:
  // Returns `true` if `record` has an acceptable latency, else `false`
  bool hasAcceptableLatency(const Record *record);

  using TimeWindows = std::unordered_map<std::string, Core::TimeWindow>;
  // Feed data to template processors
  bool feed(const Record *record);
  // Prepare detection
  void prepareResult(const linker::Association &linkerResult,
                     Result &result) const;
  // Reset the processor's processing facilities
  void resetProcessing();
  // Reset the trigger
  void resetTrigger();
  // Reset the currently enabled processors
  void resetProcessors();
  // Emit the detection result
  void emitResult(const Result &result);

  // Callback storing results from `TemplateWaveformProcessor`
  void storeTemplateResult(
      const TemplateWaveformProcessor *processor, const Record *record,
      const TemplateWaveformProcessor::MatchResultCPtr &result);

  // Callback storing results from the linker
  void storeLinkerResult(const linker::Association &linkerResult);

 private:
  // Safety margin for linker on hold duration
  static const Core::TimeSpan _linkerSafetyMargin;

  struct ProcessorState {
    ProcessorState(ProcessorState &&other) = default;
    ProcessorState &operator=(ProcessorState &&other) = default;

    // The sensor location w.r.t. the template waveform `processor`
    SensorLocation sensorLocation;
    // The time window fed
    // XXX(damb): The data time window fed might be different from the data
    // time window processed (e.g. due to the usage of certain waveform
    // operators). Therefore, keep track of the time window fed, too.
    Core::TimeWindow dataTimeWindowFed;
    // The template waveform reference time w.r.t. the template waveform
    // `processor`
    Core::Time templateWaveformReferenceTime;

    std::unique_ptr<TemplateWaveformProcessor> processor;
  };

  using ProcessorStates = std::unordered_map<std::string, ProcessorState>;
  ProcessorStates _processors;
  using ProcessorIdx = std::unordered_multimap<std::string, std::string>;
  ProcessorIdx _processorIdx;

  // The detector's status
  Status _status{Status::kWaitingForData};
  // The overall time window processed
  Core::TimeWindow _processed;

  // The current linker result
  boost::optional<linker::Association> _currentResult;
  // The result callback function
  boost::optional<PublishResultCallback> _resultCallback;

  boost::optional<double> _thresTriggerOn;
  boost::optional<double> _thresTriggerOff;
  boost::optional<Core::TimeSpan> _triggerDuration;
  boost::optional<Core::Time> _triggerEnd;
  // The processor used for triggering (i.e. the current reference processor)
  boost::optional<std::string> _triggerProcId;

  // The linker required for associating arrivals
  Linker _linker;
  using ResultQueue = std::deque<linker::Association>;
  ResultQueue _resultQueue;

  // Maximum data latency
  boost::optional<Core::TimeSpan> _maxLatency;
  // The configured processing chunk size
  boost::optional<Core::TimeSpan> _chunkSize;

  DataModel::OriginCPtr _origin;
};

}  // namespace detector
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_DETECTOR_DETECTOR_H_
