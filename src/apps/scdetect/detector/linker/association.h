#ifndef SCDETECT_APPS_SCDETECT_DETECTOR_LINKER_ASSOCIATION_H_
#define SCDETECT_APPS_SCDETECT_DETECTOR_LINKER_ASSOCIATION_H_

#include <functional>
#include <string>
#include <unordered_map>

#include "../arrival.h"
#include "../templatewaveformprocessor.h"

namespace Seiscomp {
namespace detect {
namespace detector {
namespace linker {

struct Association {
  struct TemplateResult {
    Arrival arrival;
    // Reference to the actual match result value
    TemplateWaveformProcessor::MatchResult::LocalMaxima::const_iterator
        resultIt;
    // Reference to the original template result
    TemplateWaveformProcessor::MatchResultCPtr matchResult;
  };

  // Associates `TemplateResult` with a processor (i.e. by means of the
  // processor's identifier)
  using ProcessorId = std::string;
  using TemplateResults = std::unordered_map<ProcessorId, TemplateResult>;
  TemplateResults results;

  // The association's fit [-1,1]
  double fit;

  // Returns the total number of associated arrivals
  size_t getArrivalCount() const;
  // Returns a string including debug information
  std::string debugString() const;
};

}  // namespace linker
}  // namespace detector
}  // namespace detect
}  // namespace Seiscomp

namespace std {

template <>
struct hash<Seiscomp::detect::detector::linker::Association::TemplateResult> {
  std::size_t operator()(
      const Seiscomp::detect::detector::linker::Association::TemplateResult &tr)
      const noexcept;
};

}  // namespace std

#endif  // SCDETECT_APPS_SCDETECT_DETECTOR_LINKER_ASSOCIATION_H_
