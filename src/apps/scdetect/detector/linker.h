#ifndef SCDETECT_APPS_SCDETECT_DETECTOR_LINKER_H_
#define SCDETECT_APPS_SCDETECT_DETECTOR_LINKER_H_

#include <map>
#include <string>
#include <unordered_map>

#include <seiscomp/core/datetime.h>
#include <seiscomp/core/timewindow.h>

#include "../processor.h"
#include "../template.h"
#include "arrival.h"
#include "pot.h"

namespace Seiscomp {
namespace detect {
namespace detector {

// Associates template results
class Linker {
public:
  Linker(const Core::TimeSpan &on_hold = 0.0,
         double arrival_offset_thres = 2.0e-6);
  virtual ~Linker();

  enum class Status { kWaitingForData, kTerminated };

  struct Result {
    // The result's fit [-1,1]
    double fit;
    // Reference processor id
    std::string ref_proc_id;

    struct TemplateResult {
      Arrival arrival;
      // Reference to the original template result
      detect::Template::MatchResultCPtr match_result;
    };

    // Associates `TemplateResult` with a processor (i.e. using the `proc_id`)
    using TemplateResults = std::unordered_map<std::string, TemplateResult>;
    TemplateResults results;

    // The result's POT
    POT pot;

    // Returns the total number of arrivals
    size_t GetArrivalCount() const;
    // Returns a string including debug information
    std::string DebugString() const;
  };

  // Sets the arrival offset threshold
  void set_thres_arrival_offset(const boost::optional<double> &thres);
  // Returns the current arrival offset threshold
  boost::optional<double> thres_arrival_offset() const;
  // Sets the result threshold
  void set_thres_result(const boost::optional<double> &thres);
  // Returns the result threshold
  boost::optional<double> thres_result() const;
  // Configures the linker with a minimum number of required arrivals before
  // issuing a result
  void set_min_arrivals(const boost::optional<size_t> &n);
  // Returns the minimum number of arrivals required for linking
  boost::optional<size_t> min_arrivals() const;
  // Sets the *on hold* duration
  void set_on_hold(const Core::TimeSpan &duration);
  // Returns the current *on hold* duration
  Core::TimeSpan on_hold() const;
  // Returns the linker's status
  Status status() const;
  // Returns the number of associated channels
  size_t GetAssociatedChannelCount() const;
  // Returns the number of associated processors
  size_t GetProcessorCount() const;

  // Register the processor `proc` associated with the template arrival
  // `arrival` for linking. `pick_offset` refers to the template waveform
  // pick offset.
  void Register(const detect::Processor *proc, const Arrival &arrival,
                const Core::TimeSpan &pick_offset);
  // Remove the processor identified by `proc_id`
  void Remove(const std::string &proc_id);
  // Reset the linker
  //
  // - drops all pending results
  void Reset();
  // Terminates the linker
  //
  // - flushes pending detections
  void Terminate();

  // Feeds the `proc`'s result `res` to the linker
  void Feed(const detect::Processor *proc,
            const detect::Processor::ResultCPtr &res);

  using PublishResultCallback = std::function<void(const Result &res)>;
  // Set the publish callback function
  void set_result_callback(const PublishResultCallback &cb);

protected:
  // Processes the `res`
  void Process(const detect::Processor *proc,
               const Result::TemplateResult &res);
  // Emit a result
  void EmitResult(const Result &res);

private:
  void CreatePOT();

  Status status_{Status::kWaitingForData};

  // Template processor
  struct Processor {
    // The template arrival associated
    Arrival arrival;
    // The template waveform pick offset
    Core::TimeSpan pick_offset;
  };

  using Processors = std::unordered_map<std::string, Processor>;
  Processors processors_;

  struct Event {
    // The time the event is considered as elapsed
    Core::Time expired;
    // The final result
    Result result;

    // Time of the reference arrival pick
    Core::Time ref_pick_time;

    // Merges the template result `res` into the event
    void MergeResult(const std::string &proc_id,
                     const Result::TemplateResult &res, const POT &pot);
    // Returns the total number of arrivals
    size_t GetArrivalCount() const;
  };

  using EventQueue = std::list<Event>;
  EventQueue queue_;

  // The reference POT
  POT pot_;
  bool pot_valid_{false};

  // The arrival offset threshold; if `boost::none` arrival offset threshold
  // validation is disabled; the default arrival offset corresponds to twice
  // the maximum accuracy `scdetect` is operating when it comes to trimming
  // waveforms (1 micro second (i.e. 1 us)).
  boost::optional<double> thres_arrival_offset_{2.0e-6};
  // The fit threshold indicating when template results are taken into
  // consideration
  boost::optional<double> thres_result_;
  // The minimum number of arrivals required in order to issue a result
  boost::optional<size_t> min_arrivals_;

  // The maximum time events are placed on hold before either being emitted or
  // dropped
  Core::TimeSpan on_hold_{0.0};

  // The result callback function
  boost::optional<PublishResultCallback> result_callback_;
};

} // namespace detector
} // namespace detect
} // namespace Seiscomp

namespace std {

template <>
struct hash<Seiscomp::detect::detector::Linker::Result::TemplateResult> {
  std::size_t operator()(
      const Seiscomp::detect::detector::Linker::Result::TemplateResult &tr)
      const noexcept;
};

} // namespace std

#endif // SCDETECT_APPS_SCDETECT_DETECTOR_LINKER_H_
