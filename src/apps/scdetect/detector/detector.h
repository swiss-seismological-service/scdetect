#ifndef SCDETECT_APPS_SCDETECT_DETECTOR_DETECTOR_H_
#define SCDETECT_APPS_SCDETECT_DETECTOR_DETECTOR_H_

#include <cmath>
#include <deque>
#include <functional>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/optional.hpp>

#include <seiscomp/core/datetime.h>
#include <seiscomp/core/record.h>
#include <seiscomp/core/timewindow.h>
#include <seiscomp/datamodel/origin.h>

#include "../exception.h"
#include "../processor.h"
#include "../waveformoperator.h"
#include "../waveformprocessor.h"
#include "arrival.h"
#include "linker.h"
#include "template.h"

namespace Seiscomp {
namespace detect {
namespace detector {

class Detector : public detect::Processor {
public:
  Detector(const detect::Processor *detector,
           const DataModel::OriginCPtr &origin);
  virtual ~Detector();

  class BaseException : public Exception {
  public:
    using Exception::Exception;
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
    std::string station_id;
  };

  struct Result {
    Core::Time origin_time;
    double fit;
    boost::optional<double> magnitude;

    struct TemplateResult {
      Arrival arrival;
      SensorLocation sensor_location;
    };

    using TemplateResults =
        std::unordered_multimap<std::string, TemplateResult>;
    TemplateResults template_results;

    size_t num_channels_used;
    size_t num_channels_associated;
    size_t num_stations_used;
    size_t num_stations_associated;
  };

  // Returns the detector's status
  Status status() const;
  // Returns the overall time window processed
  const Core::TimeWindow &processed() const;
  // Returns `true` if the detector is currently triggered, else `false`
  bool triggered() const;
  // Enables trigger duration facilities with `duration`
  void EnableTrigger(const Core::TimeSpan &duration);
  // Disables trigger duration facilities
  void DisableTrigger();
  // Set the trigger thresholds
  void set_trigger_thresholds(double trigger_on, double trigger_off = 1);
  // Set the maximum arrival offset threshold
  void set_arrival_offset_threshold(const boost::optional<double> &thres);
  // Returns the arrival offset threshold configured
  boost::optional<double> arrival_offset_threshold() const;
  // Configures the detector with a minimum number of arrivals required to
  // declare an event as a detection
  void set_min_arrivals(const boost::optional<size_t> &n);
  // Returns the minimum number of arrivals required in order to declare an
  // event as a detection
  boost::optional<size_t> min_arrivals() const;
  // Sets the maximum data latency w.r.t. `NOW`. If configured with
  // `boost::none` latency is not taken into account and thus not validated
  void set_maximum_latency(const boost::optional<Core::TimeSpan> &latency);
  // Returns the maximum allowed data latency configured
  boost::optional<Core::TimeSpan> maximum_latency() const;
  // Sets the processing chunk size. If configured with `boost::none` as much
  // data is processed as possible as a single chunk
  void set_chunk_size(const boost::optional<Core::TimeSpan> &chunk_size);
  // Returns the processing chunk size
  boost::optional<Core::TimeSpan> chunk_size() const;
  // Returns the number of registered template processors
  size_t GetProcessorCount() const;

  // Register the template waveform processor `proc` for processing buffered
  // data from `buf` where records are identified by the waveform stream
  // identifier `stream_id`. `proc` is registered together with the template
  // arrival `arrival` and the sensor location `loc`.
  void Register(std::unique_ptr<Template> &&proc,
                const std::shared_ptr<const RecordSequence> &buf,
                const std::string &stream_id, const Arrival &arrival,
                const Detector::SensorLocation &loc);
  // Removes the processors processing streams identified by `stream_id`
  void Remove(const std::string &stream_id);

  void Process(const std::string &stream_id_hint);
  // Reset the detector
  void Reset();
  // Terminates the detector
  //
  // - if triggered forces flushing the pending detection
  void Terminate();

  using PublishResultCallback = std::function<void(const Result &result)>;
  void set_result_callback(const PublishResultCallback &cb);

protected:
  using TimeWindows = std::unordered_map<std::string, Core::TimeWindow>;
  bool PrepareProcessing(TimeWindows &tws, const std::string &waveform_id_hint);
  // Feed data to template processors
  bool Feed(const TimeWindows &tws);
  // Prepare detection
  void PrepareResult(const Linker::Result &linker_res, Result &res) const;
  // Reset the processor's processing facilities
  void ResetProcessing();
  // Reset the trigger
  void ResetTrigger();
  // Reset the currently enabled processors
  void ResetProcessors();
  // Emit the detection result
  void EmitResult(const Result &res);

  // Callback storing results from `Template` processors
  void StoreTemplateResult(const Template *proc, const Record *rec,
                           const Template::MatchResultCPtr &res);

  // Callback storing results from the linker
  void StoreLinkerResult(const Linker::Result &res);

private:
  struct ProcessorState {
    ProcessorState(ProcessorState &&other) = default;
    ProcessorState &operator=(ProcessorState &&other) = default;
    // The `Template` waveform processor
    std::unique_ptr<Template> processor;
    // Reference to the record buffer
    std::shared_ptr<const RecordSequence> buffer;

    // The time window fed
    // XXX(damb): The data time window fed might be different from the data
    // time window processed (e.g. due to the usage of certain waveform
    // operators). Therefore, keep track of the time window fed, too.
    Core::TimeWindow data_time_window_fed;
    // The processor dependent chunk size used to feed data to `Template`
    // waveform processors
    boost::optional<Core::TimeSpan> chunk_size;

    // The sensor location w.r.t. the template `processor`
    SensorLocation sensor_location;
  };

  using ProcessorStates = std::unordered_map<std::string, ProcessorState>;
  ProcessorStates processors_;
  using ProcessorIdx = std::unordered_multimap<std::string, std::string>;
  ProcessorIdx processor_idx_;

  // The detector's status
  Status status_{Status::kWaitingForData};
  // The overall time window processed
  Core::TimeWindow processed_;

  // The current linker result
  boost::optional<Linker::Result> current_result_;
  // The result callback function
  boost::optional<PublishResultCallback> result_callback_;

  boost::optional<double> thres_trigger_on_;
  boost::optional<double> thres_trigger_off_;
  boost::optional<Core::TimeSpan> trigger_duration_;
  boost::optional<Core::Time> trigger_end_;
  // The processor used for triggering (i.e. the current reference processor)
  boost::optional<std::string> trigger_proc_id_;

  // The linker required for associating arrivals
  Linker linker_;
  using ResultQueue = std::deque<Linker::Result>;
  ResultQueue result_queue_;
  // Safety margin for linker on hold duration
  Core::TimeSpan linker_safety_margin_{1.0};

  // Maximum data latency
  boost::optional<Core::TimeSpan> max_latency_;
  // The configured processing chunk size
  boost::optional<Core::TimeSpan> chunk_size_;

  DataModel::OriginCPtr origin_;
};

} // namespace detector
} // namespace detect
} // namespace Seiscomp

#endif // SCDETECT_APPS_SCDETECT_DETECTOR_DETECTOR_H_
