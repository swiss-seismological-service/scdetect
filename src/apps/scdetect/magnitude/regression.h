#ifndef SCDETECT_APPS_SCDETECT_MAGNITUDE_REGRESSION_H_
#define SCDETECT_APPS_SCDETECT_MAGNITUDE_REGRESSION_H_

#include <seiscomp/datamodel/stationmagnitude.h>

#include <boost/optional/optional.hpp>
#include <cstdint>
#include <ratio>
#include <vector>

#include "../magnitudeprocessor.h"

namespace Seiscomp {
namespace detect {
namespace magnitude {

// Interface for regression magnitudes
class RegressionMagnitude : public Seiscomp::detect::MagnitudeProcessor {
 public:
  RegressionMagnitude(const std::string& id);

  // Maps an amplitude with a magnitude
  struct AmplitudeMagnitude {
    DataModel::AmplitudeCPtr amplitude;
    DataModel::StationMagnitudeCPtr magnitude;
  };
  // Adds multiple amplitude magnitude pairs which are going to be used for the
  // *amplitude-magnitude regression*
  //
  // - neither amplitudes nor magnitudes are validated to be consistent i.e. it
  // is up to the client to add consistent amplitudes and magnitudes)
  void add(const std::vector<AmplitudeMagnitude>& amplitudeMagnitudes);
  // Adds a single amplitude magnitude pair which is going to be used for the
  // *amplitude-magnitude regression*
  //
  // - neither amplitudes nor magnitudes are validated to be consistent i.e. it
  // is up to the client to add consistent amplitudes and magnitudes)
  void add(const AmplitudeMagnitude& amplitudeMagnitude);

  virtual void reset();

 protected:
  using AmplitudeMagnitudes = std::vector<AmplitudeMagnitude>;
  AmplitudeMagnitudes _amplitudeMagnitudes;
};

// Computes a magnitude by means of an *amplitude-magnitude regression* based
// on a fixed slope
class FixedSlopeRegressionMagnitude : public RegressionMagnitude {
 public:
  FixedSlopeRegressionMagnitude(const std::string& id);

 protected:
  double computeMagnitude(DataModel::Amplitude* amplitude) override;

  // Returns the slope used for calculating the *regression*
  virtual inline double slope() const = 0;

  void reset() override;

  void recomputeBMean();

  boost::optional<double> _bMean;
};

template <std::intmax_t Num, std::intmax_t Denom = 1>
class AnyFixedSlopeRegressionMagnitude : public FixedSlopeRegressionMagnitude {
 public:
  using FixedSlopeRegressionMagnitude::FixedSlopeRegressionMagnitude;

 protected:
  inline double slope() const override {
    // TODO(damb): compute at compile time
    return static_cast<double>(_slope.num) / static_cast<double>(_slope.den);
  }

 private:
  static const std::ratio<Num, Denom> _slope;
};

class MwFixedSlopeRegressionMagnitude
    : public AnyFixedSlopeRegressionMagnitude<2, 3> {
 public:
  MwFixedSlopeRegressionMagnitude(const std::string& id);
};

class MLFixedSlopeRegressionMagnitude
    : public AnyFixedSlopeRegressionMagnitude<1> {
 public:
  MLFixedSlopeRegressionMagnitude(const std::string& id);
};

}  // namespace magnitude
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_MAGNITUDE_REGRESSION_H_
