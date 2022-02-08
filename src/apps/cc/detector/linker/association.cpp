#include "association.h"

#include <seiscomp/core/datetime.h>

#include <boost/functional/hash.hpp>

namespace Seiscomp {
namespace detect {
namespace detector {
namespace linker {

bool operator==(const Association::TemplateResult &lhs,
                const Association::TemplateResult &rhs) {
  return lhs.arrival == rhs.arrival;
}
bool operator!=(const Association::TemplateResult &lhs,
                const Association::TemplateResult &rhs) {
  return !(lhs == rhs);
}

size_t Association::processorCount() const { return results.size(); }

std::string Association::debugString() const {
  return std::string{
      "score=" + std::to_string(score) +
      ", associated_results=" + std::to_string(processorCount())};
}

bool operator==(const Association &lhs, const Association &rhs) {
  return lhs.score == rhs.score && lhs.results == rhs.results;
}

bool operator!=(const Association &lhs, const Association &rhs) {
  return !(lhs == rhs);
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
