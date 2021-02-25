#ifndef SCDETECT_APPS_SCDETECT_VALIDATORS_H_
#define SCDETECT_APPS_SCDETECT_VALIDATORS_H_

namespace Seiscomp {
namespace detect {
namespace config {

bool ValidateXCorrThreshold(const double &thres);
bool ValidateArrivalOffsetThreshold(double thres);
bool ValidateMinArrivals(int n, int num_stream_configs = 0);

} // namespace config
} // namespace detect
} // namespace Seiscomp

#endif // SCDETECT_APPS_SCDETECT_VALIDATORS_H_
