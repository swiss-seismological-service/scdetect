#ifndef SCDETECT_APPS_SCDETECT_MAGNITUDE_REGRESSION_H_
#define SCDETECT_APPS_SCDETECT_MAGNITUDE_REGRESSION_H_

#include <seiscomp/datamodel/amplitude.h>
#include <seiscomp/datamodel/stationmagnitude.h>

#include <boost/algorithm/string/join.hpp>
#include <boost/optional/optional.hpp>
#include <cfenv>
#include <cmath>
#include <cstdint>
#include <ratio>
#include <unordered_map>
#include <vector>

#include "../log.h"
#include "../magnitudeprocessor.h"
#include "../util/math.h"
#include "decorator.h"

namespace Seiscomp {
namespace detect {
namespace magnitude {

// Convert waveform stream identifiers from an amplitude
boost::optional<std::string> getSensorLocationStreamIdFromAmplitude(
    const DataModel::Amplitude* amplitude);

/* ------------------------------------------------------------------------- */
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
  virtual void add(const std::vector<AmplitudeMagnitude>& amplitudeMagnitudes);
  // Adds a single amplitude magnitude pair which is going to be used for the
  // *amplitude-magnitude regression*
  //
  // - neither amplitudes nor magnitudes are validated to be consistent i.e. it
  // is up to the client to add consistent amplitudes and magnitudes)
  virtual void add(const AmplitudeMagnitude& amplitudeMagnitude);

  virtual void reset();

 protected:
  using AmplitudeMagnitudes = std::vector<AmplitudeMagnitude>;
  AmplitudeMagnitudes _amplitudeMagnitudes;
};

// Computes a magnitude by means of an *amplitude-magnitude regression* based
// on a fixed slope
template <std::intmax_t Num, std::intmax_t Denom = 1>
class FixedSlopeRegressionMagnitude : public RegressionMagnitude {
 public:
  FixedSlopeRegressionMagnitude(const std::string& id)
      : RegressionMagnitude{id} {}

  void add(
      const std::vector<AmplitudeMagnitude>& amplitudeMagnitudes) override {
    RegressionMagnitude::add(amplitudeMagnitudes);
    if (!amplitudeMagnitudes.empty()) {
      _bMean = boost::none;
    }
  }
  void add(const AmplitudeMagnitude& amplitudeMagnitude) override {
    RegressionMagnitude::add(amplitudeMagnitude);
    _bMean = boost::none;
  }

 protected:
  double computeMagnitude(const DataModel::Amplitude* amplitude) override {
    if (!_bMean) {
      recomputeBMean();
    }
    return slope() * amplitude->amplitude().value() + *_bMean;
  }

  // Returns the slope used for calculating the *regression*
  inline double slope() const {
    return static_cast<double>(_slope.num) / static_cast<double>(_slope.den);
  }

  void reset() override {
    RegressionMagnitude::reset();
    _bMean = boost::none;
  }

  void recomputeBMean() {
    // Given the line formula y = m*x + b and a fixed m for a set of lines (i.e.
    // both x and y are vectors), the estimate for the interseption b is
    // computed by:
    //
    //  b = mean(y-m*x)
    //
    // - currently, the implementation does not take any weighting into account
    std::feclearexcept(FE_ALL_EXCEPT);

    std::vector<double> b;
    for (const auto& ampMag : _amplitudeMagnitudes) {
      const auto& amp{ampMag.amplitude};
      const auto& mag{ampMag.magnitude};

      const double x{std::log10(amp->amplitude().value())};
      if (!std::isfinite(x)) {
        continue;
      }
      b.push_back(mag->magnitude().value() - slope() * x);
    }

    int fe{std::fetestexcept(FE_ALL_EXCEPT)};
    if ((fe & ~FE_INEXACT) != 0)  // we don't care about FE_INEXACT
    {
      std::vector<std::string> exceptions;
      if (fe & FE_DIVBYZERO) exceptions.push_back("FE_DIVBYZERO");
      if (fe & FE_INVALID) exceptions.push_back("FE_INVALID");
      if (fe & FE_OVERFLOW) exceptions.push_back("FE_OVERFLOW");
      if (fe & FE_UNDERFLOW) exceptions.push_back("FE_UNDERFLOW");

      std::string msg{
          "Floating point exception during amplitude-magnitude regression: "};
      msg += boost::algorithm::join(exceptions, ", ");
      SCDETECT_LOG_WARNING(msg.c_str());
    }

    if (b.empty()) {
      throw MagnitudeProcessor::BaseException{"missing data"};
    }

    _bMean = util::cma(b.data(), b.size());
  }

  boost::optional<double> _bMean;

 private:
  static const std::ratio<Num, Denom> _slope;
};

class MwFixedSlopeRegressionMagnitude
    : public FixedSlopeRegressionMagnitude<2, 3> {
 public:
  MwFixedSlopeRegressionMagnitude(const std::string& id);
};

class MLFixedSlopeRegressionMagnitude
    : public FixedSlopeRegressionMagnitude<1> {
 public:
  MLFixedSlopeRegressionMagnitude(const std::string& id);
};

/* ------------------------------------------------------------------------- */
class MagnitudeRange : public Decorator {
 public:
  MagnitudeRange(MagnitudeProcessor* processor, const std::string& id = "");

  class MagnitudeOutOfRange : public MagnitudeProcessor::BaseException {
   public:
    using BaseException::BaseException;
    MagnitudeOutOfRange();
  };

  // Configure a magnitude validity range with regard to magnitudes associated
  // with `detectorId`
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

}  // namespace magnitude
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_MAGNITUDE_REGRESSION_H_
