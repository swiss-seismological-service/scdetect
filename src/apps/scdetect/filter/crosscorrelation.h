#ifndef SCDETECT_APPS_SCDETECT_FILTER_CROSSCORRELATION_H_
#define SCDETECT_APPS_SCDETECT_FILTER_CROSSCORRELATION_H_

#include <boost/circular_buffer.hpp>

#include <seiscomp/core/genericrecord.h>
#include <seiscomp/core/typedarray.h>

namespace Seiscomp {
namespace detect {
namespace filter {

// Cross-Correlation filter implementation
// - has a filter delay proportional to the length of the template waveform
template <typename TData> class CrossCorrelation {
public:
  // Create a cross-correlation filter from `template_wf`. It is a bug if
  // `template_wf` is either not a valid pointer or a `nullptr`.
  CrossCorrelation(const GenericRecordCPtr &template_wf,
                   double sampling_freq = 0);

  virtual ~CrossCorrelation();

  // Apply the cross-correlation in place to the (previously filtered) data
  void Apply(size_t n_data, TData *data);

  void Apply(std::vector<TData> &data);

  void Apply(TypedArray<TData> &data);
  // Reset the cross-correlation filter
  void Reset();

  // Set the sampling frequency in Hz. Allows delayed initialization when the
  // data arrive
  void set_sampling_frequency(double sampling_freq);
  // Returns the configured sampling frequency
  double sampling_frequency() const;

  // Returns the number of template samples
  size_t template_size() const;
  // Returns the template duration in seconds
  double template_length() const;

protected:
  // Resample `wf` to `sampling_freq`
  void Resample(const GenericRecordPtr &wf, double sampling_freq);

private:
  // Filter sampling frequency_
  double sampling_frequency_{0};

  // The template waveform
  GenericRecordCPtr template_wf_;

  // Template waveform samples squared summed
  double sum_squared_template_wf_{0};
  // Template waveform samples summed
  double sum_template_wf_{0};

  double denominator_template_wf_{0};

  // The data samples squared summed
  double sum_squared_data_{0};
  // The data samples summed
  double sum_data_{0};

  // Buffer for data to be cross-correlated
  boost::circular_buffer<TData> buffer_;
};

} // namespace filter
} // namespace detect
} // namespace Seiscomp

#include "crosscorrelation.ipp"

#endif // SCDETECT_APPS_SCDETECT_FILTER_CROSSCORRELATION_H_
