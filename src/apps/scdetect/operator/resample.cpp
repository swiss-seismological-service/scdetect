#include "resample.h"

namespace Seiscomp {
namespace detect {
namespace waveform_operator {

ResamplingOperator::ResamplingOperator(
    std::unique_ptr<RecordResampler> record_resampler)
    : record_resampler_{std::move(record_resampler)} {}

WaveformProcessor::Status ResamplingOperator::Feed(const Record *record) {
  if (record->sampleCount() == 0)
    return WaveformProcessor::Status::kWaitingForData;

  auto resampled{record_resampler_->feed(record)};
  if (resampled) {
    WaveformOperator::Store(resampled);

    return WaveformProcessor::Status::kInProgress;
  }

  return WaveformProcessor::Status::kError;
}

void ResamplingOperator::Reset() { record_resampler_->reset(); }

} // namespace waveform_operator
} // namespace detect
} // namespace Seiscomp
