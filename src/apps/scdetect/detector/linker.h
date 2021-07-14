#ifndef SCDETECT_APPS_SCDETECT_DETECTOR_LINKER_H_
#define SCDETECT_APPS_SCDETECT_DETECTOR_LINKER_H_

#include <seiscomp/core/datetime.h>
#include <seiscomp/core/timewindow.h>

#include <map>
#include <memory>
#include <string>
#include <unordered_map>

#include "arrival.h"
#include "pot.h"
#include "templatewaveformprocessor.h"

namespace Seiscomp {
namespace detect {
namespace detector {

// Associates template results
class Linker {
 public:
  Linker(const Core::TimeSpan &onHold = 0.0,
         double arrivalOffsetThres = 2.0e-6);
  virtual ~Linker();

  enum class Status { kWaitingForData, kTerminated };

  struct Result {
    // The result's fit [-1,1]
    double fit;
    // Reference waveform processor id
    std::string refProcId;

    struct TemplateResult {
      Arrival arrival;
      // Reference to the original template result
      TemplateWaveformProcessor::MatchResultCPtr matchResult;
    };

    // Associates `TemplateResult` with a processor (i.e. using the `proc_id`)
    using TemplateResults = std::unordered_map<std::string, TemplateResult>;
    TemplateResults results;

    // The result's POT
    POT pot;

    // Returns the total number of arrivals
    size_t getArrivalCount() const;
    // Returns a string including debug information
    std::string debugString() const;
  };

  // Sets the arrival offset threshold
  void setThresArrivalOffset(const boost::optional<double> &thres);
  // Returns the current arrival offset threshold
  boost::optional<double> thresArrivalOffset() const;
  // Sets the result threshold
  void setThresResult(const boost::optional<double> &thres);
  // Returns the result threshold
  boost::optional<double> thresResult() const;
  // Configures the linker with a minimum number of required arrivals before
  // issuing a result
  void setMinArrivals(const boost::optional<size_t> &n);
  // Returns the minimum number of arrivals required for linking
  boost::optional<size_t> minArrivals() const;
  // Sets the *on hold* duration
  void setOnHold(const Core::TimeSpan &duration);
  // Returns the current *on hold* duration
  Core::TimeSpan onHold() const;
  // Returns the linker's status
  Status status() const;
  // Returns the number of associated channels
  size_t getAssociatedChannelCount() const;
  // Returns the number of associated processors
  size_t getProcessorCount() const;

  // Register the template waveform processor `proc` associated with the
  // template arrival `arrival` for linking.
  void add(const TemplateWaveformProcessor *proc, const Arrival &arrival);
  // Remove the processor identified by `procId`
  void remove(const std::string &procId);
  // Reset the linker
  //
  // - drops all pending results
  void reset();
  // Terminates the linker
  //
  // - flushes pending detections
  void terminate();

  // Feeds the `proc`'s result `res` to the linker
  void feed(const TemplateWaveformProcessor *proc,
            const TemplateWaveformProcessor::MatchResultCPtr &res);

  using PublishResultCallback = std::function<void(const Result &res)>;
  // Set the publish callback function
  void setResultCallback(const PublishResultCallback &callback);

 protected:
  // Processes the result `res` from `proc`
  void process(const TemplateWaveformProcessor *proc,
               const Result::TemplateResult &res);
  // Emit a result
  void emitResult(const Result &res);

 private:
  void createPot();

  Status _status{Status::kWaitingForData};

  // `TemplateWaveformProcessor` processor
  struct Processor {
    const TemplateWaveformProcessor *proc;
    // The template arrival associated
    Arrival arrival;
  };

  using Processors = std::unordered_map<std::string, Processor>;
  Processors _processors;

  struct Event {
    // The time the event is considered as elapsed
    Core::Time expired;
    // The final result
    Result result;

    // Time of the reference arrival pick
    Core::Time refPickTime;

    // Merges the template result `res` into the event
    void mergeResult(const std::string &procId,
                     const Result::TemplateResult &res, const POT &pot);
    // Returns the total number of arrivals
    size_t getArrivalCount() const;
  };

  using EventQueue = std::list<Event>;
  EventQueue _queue;

  // The reference POT
  POT _pot;
  bool _potValid{false};

  // The arrival offset threshold; if `boost::none` arrival offset threshold
  // validation is disabled; the default arrival offset corresponds to twice
  // the maximum accuracy `scdetect` is operating when it comes to trimming
  // waveforms (1 micro second (i.e. 1 us)).
  boost::optional<double> _thresArrivalOffset{2.0e-6};
  // The fit threshold indicating when template results are taken into
  // consideration
  boost::optional<double> _thresResult;
  // The minimum number of arrivals required in order to issue a result
  boost::optional<size_t> _minArrivals;

  // The maximum time events are placed on hold before either being emitted or
  // dropped
  Core::TimeSpan _onHold{0.0};

  // The result callback function
  boost::optional<PublishResultCallback> _resultCallback;
};

}  // namespace detector
}  // namespace detect
}  // namespace Seiscomp

namespace std {

template <>
struct hash<Seiscomp::detect::detector::Linker::Result::TemplateResult> {
  std::size_t operator()(
      const Seiscomp::detect::detector::Linker::Result::TemplateResult &tr)
      const noexcept;
};

}  // namespace std

#endif  // SCDETECT_APPS_SCDETECT_DETECTOR_LINKER_H_
