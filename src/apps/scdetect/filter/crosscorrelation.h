#ifndef SCDETECT_APPS_SCDETECT_FILTER_CROSSCORRELATION_H_
#define SCDETECT_APPS_SCDETECT_FILTER_CROSSCORRELATION_H_

#include <seiscomp/core/datetime.h>
#include <seiscomp/core/genericrecord.h>
#include <seiscomp/core/typedarray.h>

#include <boost/circular_buffer.hpp>
#include <boost/optional/optional.hpp>
#include <string>

#include "../template_waveform.h"

namespace Seiscomp {
namespace detect {
namespace filter {

// Cross-correlation filter implementation
//
// - the filter delay corresponds to the length of the template waveform
// - automatically adopts to different sampling frequencies (i.e. implements
// template waveform resampling facilities)
template <typename TData>
class CrossCorrelation {
 public:
  // Creates a `CrossCorrelation` filter from `waveform`. The filter is
  // configured to the sampling frequency provided by `waveform`.
  //
  // - It is a bug if `waveform` is not a valid pointer.
  explicit CrossCorrelation(const GenericRecordCPtr &waveform);
  // Creates an `CrossCorrelation` filter from `templateWaveform`
  explicit CrossCorrelation(const TemplateWaveform &templateWaveform);

  virtual ~CrossCorrelation() = default;

  // Apply the cross-correlation in place to the (previously filtered) data.
  // Before using the filter make sure the sam
  void apply(size_t nData, TData *data);

  void apply(std::vector<TData> &data);

  void apply(TypedArray<TData> &data);
  // Reset the cross-correlation filter
  virtual void reset();

  // Set the sampling frequency in Hz. Allows delayed initialization when the
  // data arrive
  void setSamplingFrequency(double sampling_frequency);
  // Returns the configured sampling frequency
  double samplingFrequency() const;

  const TemplateWaveform &templateWaveform() const;

 protected:
  // Compute the actual cross-correlation
  virtual void correlate(size_t nData, TData *data);

  virtual void setupFilter(double samplingFrequency);

 private:
  // The template waveform
  TemplateWaveform _templateWaveform;
  // Buffer for data to be cross-correlated
  boost::circular_buffer<TData> _buffer;

  // Template waveform samples squared summed
  double _sumSquaredTemplateWaveform{0};
  // Template waveform samples summed
  double _sumTemplateWaveform{0};

  double _denominatorTemplateWaveform{0};

  // The data samples squared summed
  double _sumSquaredData{0};
  // The data samples summed
  double _sumData{0};

  bool _initialized{false};
};

}  // namespace filter
}  // namespace detect
}  // namespace Seiscomp

#include "crosscorrelation.ipp"

#endif  // SCDETECT_APPS_SCDETECT_FILTER_CROSSCORRELATION_H_
