#ifndef SCDETECT_APPS_SCDETECT_MAGNITUDE_DECORATOR_RANGE_H_
#define SCDETECT_APPS_SCDETECT_MAGNITUDE_DECORATOR_RANGE_H_

#include <seiscomp/datamodel/amplitude.h>

#include <boost/optional/optional.hpp>
#include <memory>
#include <string>
#include <unordered_map>

#include "../decorator.h"

namespace Seiscomp {
namespace detect {

namespace magnitude {
namespace decorator {

class MagnitudeRange : public magnitude::Decorator {
 public:
  using Decorator::Decorator;

  class MagnitudeOutOfRange : public MagnitudeProcessor::BaseException {
   public:
    using BaseException::BaseException;
    MagnitudeOutOfRange();
  };

  double compute(const DataModel::Amplitude* amplitude) override;

  // Configure a magnitude validity range with regard to magnitudes associated
  // with `detectorId` and `sensorLocationId`
  void addLimits(const std::string& detectorId,
                 const std::string& sensorLocationId,
                 const boost::optional<double>& lower,
                 const boost::optional<double> upper);

 protected:
  double computeMagnitude(const DataModel::Amplitude* amplitude) override;
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
