#include "waveform_operator.h"

namespace Seiscomp {
namespace detect {
namespace processing {

void WaveformOperator::setStoreCallback(
    WaveformOperator::StoreCallback callback) {
  _storeCallback = std::move(callback);
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
