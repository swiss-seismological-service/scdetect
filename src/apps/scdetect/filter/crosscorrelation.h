#ifndef SCDETECT_APPS_SCDETECT_FILTER_CROSSCORRELATION_H_
#define SCDETECT_APPS_SCDETECT_FILTER_CROSSCORRELATION_H_

#include <seiscomp/core/datetime.h>
#include <seiscomp/core/genericrecord.h>
#include <seiscomp/core/typedarray.h>

#include <boost/circular_buffer.hpp>
#include <boost/optional.hpp>

namespace Seiscomp {
namespace detect {
namespace filter {

// Cross-correlation filter implementation
// - the filter delay corresponds to the length of the template waveform
template <typename TData>
class CrossCorrelation {
 public:
  // Creates a `CrossCorrelation` filter from `waveform`. The filter is
  // configured to the sampling frequency provided by `waveform`. It is a bug
  // if `waveform` is not a valid pointer.
  CrossCorrelation(const GenericRecordCPtr &waveform);

  virtual ~CrossCorrelation();

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

  // Returns the number of template samples
  size_t templateSize() const;
  // Returns the template duration in seconds
  double templateLength() const;
  // Returns a pointer to the current template waveform or `nullptr` if no
  // template waveform is available
  const GenericRecordCPtr &templateWaveform() const;

  // Returns the template waveform starttime which might be different from the
  // starttime configured (due to both sampling rate accuracy and rounding)
  boost::optional<const Core::Time> templateStartTime() const;
  // Returns the template waveform endtime which might be different from the
  // starttime configured (due to both sampling rate accuracy and rounding)
  boost::optional<const Core::Time> templateEndTime() const;

 protected:
  CrossCorrelation();

  // Compute the actual cross-correlation
  virtual void correlate(size_t nData, TData *data);

  virtual void setupFilter(double samplingFrequency);

  bool _initialized{false};
  // The template waveform to be correlated
  GenericRecordCPtr _templateWaveform;
  // Filter sampling frequency
  boost::optional<double> _samplingFrequency;
  // Buffer for data to be cross-correlated
  boost::circular_buffer<TData> _buffer;

 private:
  // Template waveform samples squared summed
  double _sumSquaredTemplateWaveform{0};
  // Template waveform samples summed
  double _sumTemplateWaveform{0};

  double _denominatorTemplateWaveform{0};

  // The data samples squared summed
  double _sumSquaredData{0};
  // The data samples summed
  double _sumData{0};
};

// Adaptive cross-correlation filter implementation
// - the filter delay corresponds to the length of the template waveform
// - automatically adopts to different sampling frequencies (i.e. implements
// template waveform resampling facilities)
template <typename TData>
class AdaptiveCrossCorrelation : public CrossCorrelation<TData> {
 public:
  // Creates an `CrossCorrelation` filter from the *demeaned* raw `waveform`
  // chunk. The final waveform used for template matching is created on-the-fly
  // based on `filterId`, `templateStartTime`, `templateEndTime` and the
  // configured target `samplingFrequency`. It is a bug if `waveform` is not a
  // valid pointer.
  AdaptiveCrossCorrelation(const GenericRecordCPtr &waveform,
                           const std::string filterId,
                           const Core::Time &templateStartTime,
                           const Core::Time &templateEndTime,
                           double samplingFrequency = 0);

 protected:
  void setupFilter(double samplingFrequency) override;
  // Setup and prepare the template waveform
  void createTemplateWaveform(double targetFrequency);

 private:
  // The original waveform chunk the `_templateWaveform` is created from
  GenericRecordCPtr _wf;
  // The filter identifier string used for template creation
  std::string _filterId;

  // The configured template waveform starttime
  Core::Time _templateStartTime;
  // The configured template waveform endtime
  Core::Time _templateEndTime;
};

}  // namespace filter
}  // namespace detect
}  // namespace Seiscomp

#include "crosscorrelation.ipp"

#endif  // SCDETECT_APPS_SCDETECT_FILTER_CROSSCORRELATION_H_
