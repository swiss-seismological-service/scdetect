#include "association.h"

#include <seiscomp/core/datetime.h>

#include <boost/functional/hash.hpp>

namespace Seiscomp {
namespace detect {
namespace detector {
namespace linker {

size_t Association::getArrivalCount() const { return results.size(); }

std::string Association::debugString() const {
  const Core::Time startTime{
      results.at(refProcId).matchResult->timeWindow.startTime()};
  const Core::Time endTime{startTime +
                           Core::TimeSpan{pot.pickOffset().value_or(0)}};
  return std::string{"(" + startTime.iso() + " - " + endTime.iso() +
                     "): fit=" + std::to_string(fit) +
                     ", arrival_count=" + std::to_string(getArrivalCount())};
}

}  // namespace linker
}  // namespace detector
}  // namespace detect
}  // namespace Seiscomp

namespace std {

inline std::size_t
hash<Seiscomp::detect::detector::linker::Association::TemplateResult>::
operator()(const Seiscomp::detect::detector::linker::Association::TemplateResult
               &tr) const noexcept {
  std::size_t ret{0};
  boost::hash_combine(
      ret, std::hash<Seiscomp::detect::detector::Arrival>{}(tr.arrival));

  if (tr.matchResult) {
    boost::hash_combine(ret, std::hash<double>{}(tr.matchResult->coefficient));
  }

  return ret;
}

}  // namespace std
