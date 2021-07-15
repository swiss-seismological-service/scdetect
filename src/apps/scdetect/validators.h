#ifndef SCDETECT_APPS_SCDETECT_VALIDATORS_H_
#define SCDETECT_APPS_SCDETECT_VALIDATORS_H_

#include <string>
#include <vector>

namespace Seiscomp {
namespace detect {
namespace config {

static const std::vector<std::string> kValidLinkerMergingStrategies{
    "all", "greaterEqualTriggerOnThreshold"};

bool validateXCorrThreshold(const double &thres);
bool validateArrivalOffsetThreshold(double thres);
bool validateMinArrivals(int n, int numStreamConfigs = 0);
bool validateSamplingFrequency(double samplingFrequency);
bool validateFilter(const std::string &filterId, std::string &err);
bool validateLinkerMergingStrategy(const std::string &mergingStrategy);

}  // namespace config
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_VALIDATORS_H_
