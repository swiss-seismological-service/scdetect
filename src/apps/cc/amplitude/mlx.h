#ifndef SCDETECT_APPS_CC_AMPLITUDE_MLX_H_
#define SCDETECT_APPS_CC_AMPLITUDE_MLX_H_

#include "rms.h"

namespace Seiscomp {
namespace detect {
namespace amplitude {

// Computes the MLx amplitude
//
// - the amplitude is computed on velocity seismograms
// - required for *amplitude-magnitude regression* magnitudes (see
// https://doi.org/10.1029/2019JB017468)
class MLx : public RMSAmplitude {
 public:
  MLx();

  void finalize(DataModel::Amplitude *amplitude) const override;
};

}  // namespace amplitude
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_CC_AMPLITUDE_MLX_H_
