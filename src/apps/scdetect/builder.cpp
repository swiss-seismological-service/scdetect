#include "builder.h"

namespace Seiscomp {
namespace detect {
namespace builder {

BaseException::BaseException() : Exception("error while object creation") {}

NoWaveformData::NoWaveformData()
    : BaseException{"no waveform data available"} {}

NoStream::NoStream() : BaseException{"no stream data available"} {}

} // namespace builder
} // namespace detect
} // namespace Seiscomp
