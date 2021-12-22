#include "validators.h"

#include <algorithm>

#include "../processing/waveform_processor.h"
#include "../settings.h"

namespace Seiscomp {
namespace detect {
namespace config {

bool validateXCorrThreshold(const double &thres) {
  return -1 <= thres && 1 >= thres;
}

bool validateArrivalOffsetThreshold(double thres) {
  return thres < 0 || (thres >= 2.0e-6);
}

bool validateMinArrivals(int n, int numStreamConfigs) {
  if (n < 0) {
    return true;
  }

  return numStreamConfigs > 0 ? n >= 1 : n >= 1 && n <= numStreamConfigs;
}

bool validateSamplingFrequency(double samplingFrequency) {
  return samplingFrequency > 0 && samplingFrequency <= 1 / 1e-6;
}

bool validateFilter(const std::string &filterId, std::string &err) {
  auto filter{processing::WaveformProcessor::Filter::Create(filterId, &err)};
  if (!filter) {
    return false;
  }
  delete filter;
  return true;
}

bool validateLinkerMergingStrategy(const std::string &mergingStrategy) {
  return std::find(kValidLinkerMergingStrategies.begin(),
                   kValidLinkerMergingStrategies.end(),
                   mergingStrategy) != kValidLinkerMergingStrategies.end();
}

bool validateMagnitudeType(const std::string &magnitudeType) {
  return std::find(kValidMagnitudeTypes.begin(), kValidMagnitudeTypes.end(),
                   magnitudeType) != kValidMagnitudeTypes.end();
}

bool validateAmplitudeType(const std::string &amplitudeType) {
  return validateMagnitudeType(amplitudeType);
}

}  // namespace config
}  // namespace detect
}  // namespace Seiscomp
