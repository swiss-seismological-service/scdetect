#include "resample.h"

namespace Seiscomp {
namespace detect {
namespace waveform_operator {

ResamplingOperator::ResamplingOperator(
    std::unique_ptr<RecordResampler> recordResampler)
    : _recordResampler{std::move(recordResampler)} {}

processing::WaveformProcessor::Status ResamplingOperator::feed(
    const Record *record) {
  if (record->sampleCount() == 0) {
    return processing::WaveformProcessor::Status::kWaitingForData;
  }

  auto *resampled{_recordResampler->feed(record)};
  if (static_cast<bool>(resampled)) {
    WaveformOperator::store(resampled);

    return processing::WaveformProcessor::Status::kInProgress;
  }

  return processing::WaveformProcessor::Status::kError;
}

void ResamplingOperator::reset() {
  if (_recordResampler) {
    // XXX(damb): Cloning is currently the only way in order to actually reset
    // the record resampler
    _recordResampler.reset(
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-static-cast-downcast)
        static_cast<RecordResampler *>(_recordResampler->clone()));
  }
}

}  // namespace waveform_operator
}  // namespace detect
}  // namespace Seiscomp
