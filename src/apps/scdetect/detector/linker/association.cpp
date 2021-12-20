#include "association.h"

#include <seiscomp/core/datetime.h>

#include <boost/functional/hash.hpp>

namespace Seiscomp {
namespace detect {
namespace detector {
namespace linker {

size_t Association::processorCount() const { return results.size(); }

std::string Association::debugString() const {
  return std::string{"fit=" + std::to_string(fit) + ", associated_results=" +
                     std::to_string(processorCount())};
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
    boost::hash_combine(ret, std::hash<double>{}(tr.resultIt->coefficient));
  }

  return ret;
}

}  // namespace std
