#ifndef SCDETECT_APPS_SCDETECT_WAVEFORMOPERATOR_H_
#define SCDETECT_APPS_SCDETECT_WAVEFORMOPERATOR_H_

#include <functional>

#include <seiscomp/core/record.h>

#include "waveformprocessor.h"

namespace Seiscomp {
namespace detect {

class WaveformOperator {
public:
  using StoreCallback = std::function<bool(const Record *record)>;

  WaveformOperator();
  virtual ~WaveformOperator();

  // Sets the callback function
  void set_store_callback(const StoreCallback &callback);

  // Feeds `record` to the operator
  virtual WaveformProcessor::Status Feed(const Record *record) = 0;

  // Resets the `WaveformOperator`
  virtual void Reset() = 0;

protected:
  bool Store(const Record *record);

private:
  StoreCallback store_callback_;
};

} // namespace detect
} // namespace Seiscomp

#endif // SCDETECT_APPS_SCDETECT_WAVEFORMOPERATOR_H_
