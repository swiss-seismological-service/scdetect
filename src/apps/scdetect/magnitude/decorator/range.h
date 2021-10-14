#ifndef SCDETECT_APPS_SCDETECT_MAGNITUDE_DECORATOR_RANGE_H_
#define SCDETECT_APPS_SCDETECT_MAGNITUDE_DECORATOR_RANGE_H_

#include <seiscomp/datamodel/amplitude.h>

#include <boost/optional/optional.hpp>
#include <string>
#include <unordered_map>

#include "../../magnitudeprocessor.h"
#include "../decorator.h"

namespace Seiscomp {
namespace detect {
namespace magnitude {
namespace decorator {

class MagnitudeRange : public Decorator {
 public:
  MagnitudeRange(MagnitudeProcessor* processor, const std::string& id = "");

  class MagnitudeOutOfRange : public MagnitudeProcessor::BaseException {
   public:
    using BaseException::BaseException;
    MagnitudeOutOfRange();
  };

  // Configure a magnitude validity range with regard to magnitudes associated
  // with `detectorId` and `sensorLocationId`
  void add(const std::string& detectorId, const std::string& sensorLocationId,
           const boost::optional<double>& lower,
           const boost::optional<double> upper);

  // Computes the magnitude while checking for configured limits
  double compute(const DataModel::Amplitude* amplitude) override;

 protected:
  // Called in case the magnitude is out of range
  //
  // - the default implementation throws a `MagnitudeOutOfRange` exception
  virtual double handleMagnitudeOutOfRange(
      const DataModel::Amplitude* amplitude, double magnitude);

 private:
  struct Range {
    boost::optional<double> begin;
    boost::optional<double> end;
  };

  using DetectorId = std::string;
  using SensorLocationId = std::string;
  using Ranges =
      std::unordered_map<DetectorId,
                         std::unordered_map<SensorLocationId, Range>>;
  Ranges _ranges;
};

}  // namespace decorator
}  // namespace magnitude
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_MAGNITUDE_DECORATOR_RANGE_H_
