#include "waveformoperator.h"

namespace Seiscomp {
namespace detect {

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

}  // namespace detect
}  // namespace Seiscomp
