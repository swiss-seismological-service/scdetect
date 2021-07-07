#include "arrival.h"

#include <boost/functional/hash.hpp>

namespace Seiscomp {
namespace detect {
namespace detector {

bool operator==(const Pick &lhs, const Pick &rhs) {
  return (lhs.time == rhs.time && lhs.waveform_id == rhs.waveform_id &&
          lhs.phase_hint == rhs.phase_hint && lhs.offset == rhs.offset &&
          lhs.lower_uncertainty == rhs.lower_uncertainty &&
          lhs.upper_uncertainty == rhs.upper_uncertainty);
}

bool operator!=(const Pick &lhs, const Pick &rhs) { return !(lhs == rhs); }

/* ------------------------------------------------------------------------- */
Arrival::Arrival(const Pick &pick, const std::string &phase, double weight)
    : pick{pick}, phase{phase}, weight{weight} {}

bool operator==(const Arrival &lhs, const Arrival &rhs) {
  return (lhs.pick == rhs.pick && lhs.phase == rhs.phase &&
          lhs.weight == rhs.weight);
}

bool operator!=(const Arrival &lhs, const Arrival &rhs) {
  return !(lhs == rhs);
}

}  // namespace detector
}  // namespace detect
}  // namespace Seiscomp

namespace std {

inline std::size_t hash<Seiscomp::detect::detector::Arrival>::operator()(
    const Seiscomp::detect::detector::Arrival &a) const noexcept {
  std::size_t ret{0};
  boost::hash_combine(ret, std::hash<std::string>{}(a.pick.time.iso()));
  boost::hash_combine(ret, std::hash<std::string>{}(a.pick.waveform_id));
  boost::hash_combine(ret, std::hash<std::string>{}(a.phase));
  return ret;
}

}  // namespace std
