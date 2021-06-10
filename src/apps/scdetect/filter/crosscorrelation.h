#ifndef SCDETECT_APPS_SCDETECT_FILTER_CROSSCORRELATION_H_
#define SCDETECT_APPS_SCDETECT_FILTER_CROSSCORRELATION_H_

#include <boost/circular_buffer.hpp>
#include <boost/optional.hpp>

#include <seiscomp/core/datetime.h>
#include <seiscomp/core/genericrecord.h>
#include <seiscomp/core/typedarray.h>

namespace Seiscomp {
namespace detect {
namespace filter {

// Cross-correlation filter implementation
// - the filter delay corresponds to the length of the template waveform
template <typename TData> class CrossCorrelation {
public:
  // Creates a `CrossCorrelation` filter from `waveform`. The filter is
  // configured to the sampling frequency provided by `waveform`. It is a bug
  // if `waveform` is not a valid pointer.
  CrossCorrelation(const GenericRecordCPtr &waveform);

  virtual ~CrossCorrelation();

  // Apply the cross-correlation in place to the (previously filtered) data.
  // Before using the filter make sure the sam
  void Apply(size_t n_data, TData *data);

  void Apply(std::vector<TData> &data);

  void Apply(TypedArray<TData> &data);
  // Reset the cross-correlation filter
  virtual void Reset();

  // Set the sampling frequency in Hz. Allows delayed initialization when the
  // data arrive
  void set_sampling_frequency(double sampling_frequency);
  // Returns the configured sampling frequency
  double sampling_frequency() const;

  // Returns the number of template samples
  size_t template_size() const;
  // Returns the template duration in seconds
  double template_length() const;

  // Returns the template waveform starttime which might be different from the
  // starttime configured (due to both sampling rate accuracy and rounding)
  boost::optional<const Core::Time> template_starttime() const;
  // Returns the template waveform endtime which might be different from the
  // starttime configured (due to both sampling rate accuracy and rounding)
  boost::optional<const Core::Time> template_endtime() const;

protected:
  CrossCorrelation();

  // Compute the actual cross-correlation
  virtual void Correlate(size_t n_data, TData *data);

  virtual void SetupFilter(double sampling_frequency);

  bool initialized_{false};
  // The template waveform to be correlated
  GenericRecordCPtr template_wf_;
  // Filter sampling frequency
  boost::optional<double> sampling_frequency_;
  // Buffer for data to be cross-correlated
  boost::circular_buffer<TData> buffer_;

private:
  // Template waveform samples squared summed
  double sum_squared_template_wf_{0};
  // Template waveform samples summed
  double sum_template_wf_{0};

  double denominator_template_wf_{0};

  // The data samples squared summed
  double sum_squared_data_{0};
  // The data samples summed
  double sum_data_{0};
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
  // based on `filter_id`, `template_starttime`, `template_endtime` and the
  // configured target `sampling_frequency`. It is a bug if `waveform` is not a
  // valid pointer.
  AdaptiveCrossCorrelation(const GenericRecordCPtr &waveform,
                           const std::string filter_id,
                           const Core::Time &template_starttime,
                           const Core::Time &template_endtime,
                           double sampling_frequency = 0);

protected:
  void SetupFilter(double sampling_frequency) override;
  // Setup and prepare the template waveform
  void CreateTemplateWaveform(double target_frequency);

private:
  // The original waveform chunk the `template_wf_` is created from
  GenericRecordCPtr wf_;
  // The filter identifier string used for template creation
  std::string filter_id_;

  // The configued template waveform starttime
  Core::Time template_starttime_;
  // The configued template waveform endtime
  Core::Time template_endtime_;
};

} // namespace filter
} // namespace detect
} // namespace Seiscomp

#include "crosscorrelation.ipp"

#endif // SCDETECT_APPS_SCDETECT_FILTER_CROSSCORRELATION_H_
