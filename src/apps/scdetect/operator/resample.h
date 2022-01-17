#ifndef SCDETECT_APPS_SCDETECT_OPERATOR_RESAMPLE_H_
#define SCDETECT_APPS_SCDETECT_OPERATOR_RESAMPLE_H_

#include <memory>

#include "../processing/waveform_operator.h"
#include "../processing/waveform_processor.h"
#include "../resamplerstore.h"

namespace Seiscomp {
namespace detect {
namespace waveform_operator {

class ResamplingOperator : public processing::WaveformOperator {
 public:
  using RecordResampler = RecordResamplerStore::RecordResampler;

  explicit ResamplingOperator(std::unique_ptr<RecordResampler> recordResampler);

  processing::WaveformProcessor::Status feed(const Record *record) override;

  void reset() override;

 private:
  std::unique_ptr<RecordResampler> _recordResampler;
};

}  // namespace waveform_operator
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_OPERATOR_RESAMPLE_H_
