#include "regression.h"

#include <boost/algorithm/string/join.hpp>
#include <cfenv>
#include <cmath>
#include <vector>

#include "../log.h"
#include "../utils.h"

namespace Seiscomp {
namespace detect {
namespace magnitude {

RegressionMagnitude::RegressionMagnitude(const std::string& id)
    : MagnitudeProcessor{id} {}

void RegressionMagnitude::add(
    const std::vector<RegressionMagnitude::AmplitudeMagnitude>&
        amplitudeMagnitudes) {
  _amplitudeMagnitudes.insert(std::end(_amplitudeMagnitudes),
                              std::begin(amplitudeMagnitudes),
                              std::end(amplitudeMagnitudes));
}

void RegressionMagnitude::add(
    const RegressionMagnitude::AmplitudeMagnitude& amplitudeMagnitude) {
  _amplitudeMagnitudes.push_back(amplitudeMagnitude);
}

void RegressionMagnitude::reset() { _amplitudeMagnitudes.clear(); }

/* ------------------------------------------------------------------------- */
FixedSlopeRegressionMagnitude::FixedSlopeRegressionMagnitude(
    const std::string& id)
    : RegressionMagnitude{id} {}

void FixedSlopeRegressionMagnitude::reset() {
  RegressionMagnitude::reset();
  _bMean = boost::none;
}

double FixedSlopeRegressionMagnitude::computeMagnitude(
    DataModel::Amplitude* amplitude) {
  if (!_bMean) {
    recomputeBMean();
  }
  return slope() * amplitude->amplitude().value() + *_bMean;
}

void FixedSlopeRegressionMagnitude::recomputeBMean() {
  // Given the line formula y = m*x + b and a fixed m for a set of lines (i.e.
  // both x and y are vectors), the estimate for the interseption b is computed
  // by:
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

  _bMean = utils::cma(b.data(), b.size());
}

/* ------------------------------------------------------------------------- */
MwFixedSlopeRegressionMagnitude::MwFixedSlopeRegressionMagnitude(
    const std::string& id)
    : AnyFixedSlopeRegressionMagnitude{id} {
  _type = "Mw";
}

MLFixedSlopeRegressionMagnitude::MLFixedSlopeRegressionMagnitude(
    const std::string& id)
    : AnyFixedSlopeRegressionMagnitude{id} {
  _type = "ML";
}

}  // namespace magnitude
}  // namespace detect
}  // namespace Seiscomp
