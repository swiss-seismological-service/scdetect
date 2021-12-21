#ifndef SCDETECT_APPS_SCDETECT_DETECTOR_LINKER_H_
#define SCDETECT_APPS_SCDETECT_DETECTOR_LINKER_H_

#include <seiscomp/core/datetime.h>
#include <seiscomp/core/timewindow.h>

#include <boost/optional/optional.hpp>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>

#include "arrival.h"
#include "linker/association.h"
#include "linker/pot.h"
#include "linker/strategy.h"
#include "templatewaveformprocessor.h"

namespace Seiscomp {
namespace detect {
namespace detector {

// Associates `TemplateWaveformProcessor` results
class Linker {
 public:
  Linker(const Core::TimeSpan &onHold = 0.0,
         double arrivalOffsetThres = 2.0e-6);
  virtual ~Linker();

  enum class Status { kWaitingForData, kTerminated };

  // Sets the arrival offset threshold
  void setThresArrivalOffset(const boost::optional<double> &thres);
  // Returns the current arrival offset threshold
  boost::optional<double> thresArrivalOffset() const;
  // Sets the association threshold
  void setThresAssociation(const boost::optional<double> &thres);
  // Returns the association threshold
  boost::optional<double> thresAssociation() const;
  // Configures the linker with a minimum number of required arrivals before
  // issuing a result
  void setMinArrivals(const boost::optional<size_t> &n);
  // Returns the minimum number of arrivals required for linking
  boost::optional<size_t> minArrivals() const;
  // Sets the *on hold* duration
  void setOnHold(const Core::TimeSpan &duration);
  // Returns the current *on hold* duration
  Core::TimeSpan onHold() const;
  // Sets the linker's merging strategy based on `mergingStrategyTypeId`
  void setMergingStrategy(linker::MergingStrategy::Type mergingStrategyTypeId);
  // Returns the linker's status
  Status status() const;
  // Returns the number of associated channels
  size_t channelCount() const;
  // Returns the number of associated processors
  size_t processorCount() const;

  // Register the template waveform processor `proc` associated with the
  // template arrival `arrival` for linking.
  void add(const TemplateWaveformProcessor *proc, const Arrival &arrival,
           const boost::optional<double> &mergingThreshold);
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
            const TemplateWaveformProcessor::MatchResultCPtr &matchResult);

  using PublishResultCallback =
      std::function<void(const linker::Association &)>;
  // Set the publish callback function
  void setResultCallback(const PublishResultCallback &callback);

 protected:
  // Processes the result `res` from `proc`
  void process(const TemplateWaveformProcessor *proc,
               const linker::Association::TemplateResult &result);
  // Emit a result
  void emitResult(const linker::Association &result);

 private:
  void createPot();

  struct Candidate;
  linker::POT createCandidatePOT(
      const Candidate &candidate, const std::string &processorId,
      const linker::Association::TemplateResult &newResult);

  Status _status{Status::kWaitingForData};

  // `TemplateWaveformProcessor` processor
  struct Processor {
    const TemplateWaveformProcessor *proc;
    // The template arrival associated
    Arrival arrival;
    // The processor specific merging threshold
    boost::optional<double> mergingThreshold;
  };

  // Maps the processor id with `Processor`
  using Processors = std::unordered_map<std::string, Processor>;
  Processors _processors;

  struct Candidate {
    // The time after the event is considered as expired
    Core::Time expired;
    // The final association
    linker::Association association;

    Candidate(const Core::Time &expired);
    // Feeds the template result `res` to the event in order to be merged
    void feed(const std::string &procId,
              const linker::Association::TemplateResult &res);
    // Returns the number of associated processors
    size_t associatedProcessorCount() const;
    // Returns `true` if the event must be considered as expired
    bool isExpired(const Core::Time &now) const;
  };

  using CandidateQueue = std::list<Candidate>;
  CandidateQueue _queue;

  // The linker's reference POT
  linker::POT _pot;
  bool _potValid{false};

  // The arrival offset threshold; if `boost::none` arrival offset threshold
  // validation is disabled; the default arrival offset corresponds to twice
  // the maximum accuracy `scdetect` is operating when it comes to trimming
  // waveforms (1 micro second (i.e. 1 us)).
  boost::optional<double> _thresArrivalOffset{2.0e-6};
  // The association threshold indicating when template results are taken into
  // consideration
  boost::optional<double> _thresAssociation;
  // The minimum number of arrivals required in order to issue a result
  boost::optional<size_t> _minArrivals;

  // The maximum time events are placed on hold before either being emitted or
  // dropped
  Core::TimeSpan _onHold{0.0};

  // The merging strategy used while linking
  std::unique_ptr<linker::MergingStrategy> _mergingStrategy;

  // The result callback function
  boost::optional<PublishResultCallback> _resultCallback;
};

}  // namespace detector
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_DETECTOR_LINKER_H_
