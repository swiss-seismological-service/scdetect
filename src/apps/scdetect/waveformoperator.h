#ifndef SCDETECT_APPS_SCDETECT_WAVEFORMOPERATOR_H_
#define SCDETECT_APPS_SCDETECT_WAVEFORMOPERATOR_H_

#include <seiscomp/core/record.h>

#include <functional>

#include "waveformprocessor.h"

namespace Seiscomp {
namespace detect {

class WaveformOperator {
 public:
  using StoreCallback = std::function<bool(const Record *record)>;

  WaveformOperator();
  virtual ~WaveformOperator();

  // Sets the callback function
  void setStoreCallback(const StoreCallback &callback);

  // Feeds `record` to the operator
  virtual WaveformProcessor::Status feed(const Record *record) = 0;

  // Resets the `WaveformOperator`
  virtual void reset() = 0;

 protected:
  bool store(const Record *record);

 private:
  StoreCallback _storeCallback;
};

}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_WAVEFORMOPERATOR_H_
