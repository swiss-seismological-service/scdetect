#ifndef SCDETECT_APPS_SCDETECT_TEMPLATEWAVEFORM_H_
#define SCDETECT_APPS_SCDETECT_TEMPLATEWAVEFORM_H_

#include <seiscomp/core/datetime.h>
#include <seiscomp/core/genericrecord.h>

#include <string>

namespace Seiscomp {
namespace detect {

// Wraps the template waveform
class TemplateWaveform {
 public:
  // Wraps `waveform`
  //
  // - it is a bug if `waveform` is not a valid pointer
  TemplateWaveform(const GenericRecordCPtr &waveform);

  // Creates `TemplateWaveform` from the *demeaned* raw `waveform`
  // chunk. The final waveform is created on-the-fly based on
  // `templateStartTime`, `templateEndTime`, `filter` and the configured target
  // `samplingFrequency`
  //
  // - it is a bug if `waveform` is not a valid pointer
  TemplateWaveform(const GenericRecordCPtr &waveform,
                   const boost::optional<Core::Time> &templateStartTime,
                   const boost::optional<Core::Time> &templateEndTime,
                   const boost::optional<std::string> &filter,
                   boost::optional<double> samplingFrequency);

  // Returns the underlying *raw* record the actual template waveform is created
  // from
  const GenericRecord &raw() const;
  // Returns the template waveform
  const GenericRecord &waveform() const;

  // Set the sampling frequency in Hz
  void setSamplingFrequency(double samplingFrequency);
  // Returns the configured sampling frequency
  //
  // - may force the template waveform to be recreated
  double samplingFrequency() const;

  // Returns the number of template samples
  std::size_t size() const;
  // Returns the template waveform duration in seconds
  double length() const;

  // Returns the template waveform starttime which might be different from the
  // starttime configured (due to both sampling rate accuracy and rounding)
  Core::Time startTime() const;
  // Returns the template waveform endtime which might be different from the
  // starttime configured (due to both sampling rate accuracy and rounding)
  Core::Time endTime() const;

  // Resets the processed template waveform
  void reset();

 private:
  const GenericRecord &templateWaveform() const;

  GenericRecordCPtr createTemplateWaveform(
      double targetSamplingFrequency) const;

  // The template waveform
  mutable GenericRecordCPtr _templateWaveform;
  // The original waveform chunk the `_templateWaveform` is created from
  GenericRecordCPtr _raw;

  // The configured template waveform starttime
  boost::optional<Core::Time> _startTime;
  // The configured template waveform endtime
  boost::optional<Core::Time> _endTime;

  // The filter string identifier used for template creation
  boost::optional<std::string> _filter;
  // The target sampling frequency
  boost::optional<double> _samplingFrequency;
};

}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_TEMPLATEWAVEFORM_H_
