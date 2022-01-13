#ifndef SCDETECT_APPS_CC_PROCESSING_WAVEFORMOPERATOR_H_
#define SCDETECT_APPS_CC_PROCESSING_WAVEFORMOPERATOR_H_

#include <seiscomp/core/record.h>

#include <functional>

#include "waveform_processor.h"

namespace Seiscomp {
namespace detect {
namespace processing {

class WaveformOperator {
 public:
  using StoreCallback = std::function<bool(const Record *)>;

  virtual ~WaveformOperator() = default;

  // Sets the callback function
  void setStoreCallback(StoreCallback callback);

  // Feeds `record` to the operator
  //
  // - the returned status must be interpreted in the context of the `record`
  // fed (i.e. not in the context of the operator itself)
  virtual WaveformProcessor::Status feed(const Record *record) = 0;

  // Resets the `WaveformOperator`
  virtual void reset() = 0;

 protected:
  bool store(const Record *record);

 private:
  StoreCallback _storeCallback;
};

}  // namespace processing
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_CC_PROCESSING_WAVEFORMOPERATOR_H_
