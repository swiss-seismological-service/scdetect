#ifndef SCDETECT_APPS_SCDETECT_DETECTOR_RESULT_H_
#define SCDETECT_APPS_SCDETECT_DETECTOR_RESULT_H_

#include <seiscomp/core/genericrecord.h>

#include <string>

namespace Seiscomp {
namespace detect {
namespace detector {

struct DebugInfo {
  std::string processorId;
  // Reference to the filtered data to be cross-correlated
  GenericRecordCPtr waveform;
  // Reference to the template waveform chunk used for cross-correlation
  GenericRecordCPtr templateWaveform;
};

}  // namespace detector
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_DETECTOR_RESULT_H_
