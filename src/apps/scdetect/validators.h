#ifndef SCDETECT_APPS_SCDETECT_VALIDATORS_H_
#define SCDETECT_APPS_SCDETECT_VALIDATORS_H_

#include <string>

namespace Seiscomp {
namespace detect {
namespace config {

bool validateXCorrThreshold(const double &thres);
bool validateArrivalOffsetThreshold(double thres);
bool validateMinArrivals(int n, int numStreamConfigs = 0);
bool validateSamplingFrequency(double samplingFrequency);
bool validateFilter(const std::string &filterId, std::string &err);

}  // namespace config
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_VALIDATORS_H_
