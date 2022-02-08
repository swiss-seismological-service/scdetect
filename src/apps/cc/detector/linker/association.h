#ifndef SCDETECT_APPS_CC_DETECTOR_LINKER_ASSOCIATION_H_
#define SCDETECT_APPS_CC_DETECTOR_LINKER_ASSOCIATION_H_

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "../arrival.h"
#include "../template_waveform_processor.h"

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
    std::shared_ptr<const TemplateWaveformProcessor::MatchResult> matchResult;

    friend bool operator==(const TemplateResult &lhs,
                           const TemplateResult &rhs);
    friend bool operator!=(const TemplateResult &lhs,
                           const TemplateResult &rhs);
  };

  // Associates `TemplateResult` with a processor (i.e. by means of the
  // processor's identifier)
  using ProcessorId = std::string;
  using TemplateResults = std::map<ProcessorId, TemplateResult>;
  TemplateResults results;

  // The association's score [-1,1]
  double score;

  // Returns the total number of associated processors
  std::size_t processorCount() const;
  // Returns a string including debug information
  std::string debugString() const;

  friend bool operator==(const Association &lhs, const Association &rhs);
  friend bool operator!=(const Association &lhs, const Association &rhs);
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

#endif  // SCDETECT_APPS_CC_DETECTOR_LINKER_ASSOCIATION_H_
