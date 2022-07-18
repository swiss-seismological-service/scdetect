#ifndef SCDETECT_APPS_CC_AMPLITUDE_UTIL_H_
#define SCDETECT_APPS_CC_AMPLITUDE_UTIL_H_

#include <seiscomp/datamodel/amplitude.h>
#include <seiscomp/datamodel/comment.h>
#include <seiscomp/datamodel/waveformstreamid.h>

#include <memory>

namespace Seiscomp {
namespace detect {

class AmplitudeProcessor;

namespace amplitude {
namespace util {

std::unique_ptr<DataModel::Comment> createAssociatedWaveformStreamIdComment(
    const AmplitudeProcessor* amplitudeProcessor);

std::unique_ptr<DataModel::Comment> createDetectorIdComment(
    const AmplitudeProcessor* amplitudeProcessor);

void setWaveformStreamId(const AmplitudeProcessor* amplitudeProcessor,
                         DataModel::Amplitude& amplitude);

}  // namespace util
}  // namespace amplitude
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_CC_AMPLITUDE_UTIL_H_
