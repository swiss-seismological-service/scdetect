#ifndef SCDETECT_APPS_CC_CONFIG_VALIDATORS_H_
#define SCDETECT_APPS_CC_CONFIG_VALIDATORS_H_

#include <string>
#include <vector>

namespace Seiscomp {
namespace detect {
namespace config {

static const std::vector<std::string> kValidLinkerMergingStrategies{
    "all", "greaterEqualTriggerOnThreshold", "greaterEqualMergingThreshold"};

static const std::vector<std::string> kValidMagnitudeTypes{"MRelative", "MLx"};

bool validateXCorrThreshold(const double &thres);
bool validateArrivalOffsetThreshold(double thres);
bool validateMinArrivals(int n, int numStreamConfigs = 0);
bool validateSamplingFrequency(double samplingFrequency);
bool validateFilter(const std::string &filterId, std::string &err);
bool validateLinkerMergingStrategy(const std::string &mergingStrategy);
bool validateMagnitudeType(const std::string &magnitudeType);
bool validateAmplitudeType(const std::string &amplitudeType);

}  // namespace config
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_CC_CONFIG_VALIDATORS_H_
