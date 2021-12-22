#include "waveform_operator.h"

namespace Seiscomp {
namespace detect {
namespace processing {

WaveformOperator::WaveformOperator() {}

WaveformOperator::~WaveformOperator() {}

void WaveformOperator::setStoreCallback(
    const WaveformOperator::StoreCallback &callback) {
  _storeCallback = callback;
}

bool WaveformOperator::store(const Record *record) {
  if (_storeCallback) {
    return _storeCallback(record);
  }
  return false;
}

}  // namespace processing
}  // namespace detect
}  // namespace Seiscomp
