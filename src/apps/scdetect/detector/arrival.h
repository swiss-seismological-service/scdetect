#ifndef SCDETECT_APPS_SCDETECT_DETECTOR_ARRIVAL_H_
#define SCDETECT_APPS_SCDETECT_DETECTOR_ARRIVAL_H_

#include <seiscomp/core/datetime.h>

#include <boost/optional.hpp>
#include <functional>
#include <string>

namespace Seiscomp {
namespace detect {
namespace detector {

// A detector pick
struct Pick {
  // The pick's time
  Core::Time time;
  // The pick's waveform stream identifier
  std::string waveform_id;

  // The tentative phase
  boost::optional<std::string> phase_hint;

  // The pick offset w.r.t. origin time
  Core::TimeSpan offset;

  // Lower uncertainty w.r.t. the pick time
  boost::optional<double> lower_uncertainty;
  // Upper uncertainty w.r.t. the pick time
  boost::optional<double> upper_uncertainty;

  friend bool operator==(const Pick &lhs, const Pick &rhs);
  friend bool operator!=(const Pick &lhs, const Pick &rhs);
};

// A detector arrival
struct Arrival {
  Arrival(const Pick &pick, const std::string &phase, double weight = 0);

  // The associated pick
  Pick pick;
  // The associated phase code
  std::string phase;
  // The arrival weight
  double weight{0};

  friend bool operator==(const Arrival &lhs, const Arrival &rhs);
  friend bool operator!=(const Arrival &lhs, const Arrival &rhs);
};

}  // namespace detector
}  // namespace detect
}  // namespace Seiscomp

namespace std {

template <>
struct hash<Seiscomp::detect::detector::Arrival> {
  std::size_t operator()(
      const Seiscomp::detect::detector::Arrival &a) const noexcept;
};

}  // namespace std

#endif  // SCDETECT_APPS_SCDETECT_DETECTOR_ARRIVAL_H_
