#ifndef SCDETECT_APPS_CC_DETECTOR_ARRIVAL_IPP_
#define SCDETECT_APPS_CC_DETECTOR_ARRIVAL_IPP_

#include <boost/functional/hash.hpp>

namespace std {

inline std::size_t hash<Seiscomp::detect::detector::Pick>::operator()(
    const Seiscomp::detect::detector::Pick &p) const noexcept {
  std::size_t ret{0};
  boost::hash_combine(ret, std::hash<std::string>{}(p.time.iso()));
  boost::hash_combine(ret, std::hash<std::string>{}(p.waveformStreamId));
  if (p.phaseHint) {
    boost::hash_combine(ret, std::hash<std::string>{}(*p.phaseHint));
  }
  boost::hash_combine(ret, std::hash<double>{}(p.offset.length()));
  if (p.lowerUncertainty) {
    boost::hash_combine(ret, std::hash<double>{}(*p.lowerUncertainty));
  }
  if (p.upperUncertainty) {
    boost::hash_combine(ret, std::hash<double>{}(*p.upperUncertainty));
  }
  return ret;
}

inline std::size_t hash<Seiscomp::detect::detector::Arrival>::operator()(
    const Seiscomp::detect::detector::Arrival &a) const noexcept {
  std::size_t ret{0};
  boost::hash_combine(ret,
                      std::hash<Seiscomp::detect::detector::Pick>{}(a.pick));
  boost::hash_combine(ret, std::hash<std::string>{}(a.phase));
  boost::hash_combine(ret, std::hash<double>{}(a.weight));
  return ret;
}

}  // namespace std

#endif
