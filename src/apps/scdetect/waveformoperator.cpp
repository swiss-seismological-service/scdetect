#include "waveformoperator.h"

namespace Seiscomp {
namespace detect {

WaveformOperator::WaveformOperator() {}

WaveformOperator::~WaveformOperator() {}

void WaveformOperator::set_store_callback(
    const WaveformOperator::StoreCallback &callback) {
  store_callback_ = callback;
}

bool WaveformOperator::Store(const Record *record) {
  if (store_callback_) {
    return store_callback_(record);
  }
  return false;
}

} // namespace detect
} // namespace Seiscomp
