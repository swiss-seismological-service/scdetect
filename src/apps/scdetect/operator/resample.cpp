#include "resample.h"

namespace Seiscomp {
namespace detect {
namespace waveform_operator {

ResamplingOperator::ResamplingOperator(
    std::unique_ptr<RecordResampler> recordResampler)
    : _recordResampler{std::move(recordResampler)} {}

WaveformProcessor::Status ResamplingOperator::feed(const Record *record) {
  if (record->sampleCount() == 0)
    return WaveformProcessor::Status::kWaitingForData;

  auto resampled{_recordResampler->feed(record)};
  if (resampled) {
    WaveformOperator::store(resampled);

    return WaveformProcessor::Status::kInProgress;
  }

  return WaveformProcessor::Status::kError;
}

void ResamplingOperator::reset() { _recordResampler->reset(); }

}  // namespace waveform_operator
}  // namespace detect
}  // namespace Seiscomp
