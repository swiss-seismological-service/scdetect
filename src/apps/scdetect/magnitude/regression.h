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
#include <vector>

#include "../log.h"
#include "../magnitudeprocessor.h"
#include "../util/math.h"
#include "templatefamily.h"

namespace Seiscomp {
namespace detect {
namespace magnitude {

// Computes a magnitude by means of an *amplitude-magnitude regression* based
// on a fixed slope
template <std::intmax_t Num, std::intmax_t Denom = 1>
class FixedSlopeRegressionMagnitude : public MagnitudeProcessor,
                                      public TemplateFamilyBased {
 public:
  void addAmplitudeMagnitude(
      DataModel::AmplitudeCPtr amplitude,
      DataModel::StationMagnitudeCPtr magnitude) override {
    TemplateFamilyBased::addAmplitudeMagnitude(amplitude, magnitude);
    _bMean = boost::none;
  }

  void resetAmplitudeMagnitudes() override {
    TemplateFamilyBased::resetAmplitudeMagnitudes();
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
    for (const auto& ampMag : *this) {
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

class MwxFixedSlopeRegressionMagnitude
    : public FixedSlopeRegressionMagnitude<2, 3> {
 public:
  MwxFixedSlopeRegressionMagnitude();
};

class MLxFixedSlopeRegressionMagnitude
    : public FixedSlopeRegressionMagnitude<1> {
 public:
  MLxFixedSlopeRegressionMagnitude();
};

}  // namespace magnitude
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_MAGNITUDE_REGRESSION_H_
