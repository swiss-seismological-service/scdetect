#ifndef SCDETECT_APPS_SCDETECT_TEMPLATEWAVEFORM_H_
#define SCDETECT_APPS_SCDETECT_TEMPLATEWAVEFORM_H_

#include <seiscomp/core/datetime.h>
#include <seiscomp/core/genericrecord.h>

#include <boost/optional/optional.hpp>
#include <functional>
#include <string>

#include "waveform.h"

namespace Seiscomp {
namespace detect {

// Wraps the template waveform
class TemplateWaveform {
 public:
  // Template waveform processing configuration
  struct ProcessingConfig {
    // The template waveform starttime
    boost::optional<Core::Time> templateStartTime;
    // The template waveform endtime
    boost::optional<Core::Time> templateEndTime;

    boost::optional<std::string> filter;
    // The filter initialization time
    boost::optional<Core::TimeSpan> initTime;

    // A raw waveform safety margin
    boost::optional<Core::TimeSpan> safetyMargin;

    // The sampling frequency in Hz
    boost::optional<double> samplingFrequency;

    bool detrend{false};
    bool demean{false};
  };

  using ProcessingStrategy = std::function<GenericRecordCPtr(
      const GenericRecordCPtr &, const ProcessingConfig &)>;

  static const ProcessingStrategy noProcessing;
  static const ProcessingStrategy defaultProcessing;

  // Wraps `waveform`
  explicit TemplateWaveform(const GenericRecordCPtr &waveform = nullptr);
  // Creates `TemplateWaveform` from the raw `waveform`
  // chunk. The final waveform is created on-the-fly based on
  // `processingConfig` and `processingStrategy`.
  TemplateWaveform(
      const GenericRecordCPtr &waveform,
      const ProcessingConfig &processingConfig,
      const ProcessingStrategy &processingStrategy = defaultProcessing);

  // Loads the raw waveform by means of the `waveformHandler`
  static TemplateWaveform load(
      WaveformHandlerIface *waveformHandler, const std::string &netCode,
      const std::string &staCode, const std::string &locCode,
      const std::string &chaCode, const ProcessingConfig &processingConfig,
      const ProcessingStrategy &processingStrategy = defaultProcessing);

  // Returns the underlying *raw* record the actual template waveform is created
  // from
  const GenericRecord &raw() const;
  // Sets the underlying raw waveform
  void setRaw(const GenericRecordCPtr &raw);
  // Returns the template waveform
  const GenericRecord &waveform() const;

  // Returns the template waveform stream identifier
  std::string waveformStreamId() const;

  // Returns the processing configuration
  const ProcessingConfig &processingConfig() const;
  // Sets the processing config
  //
  // - forces the template waveform to be recreated
  void setProcessingConfig(const ProcessingConfig &config);
  // Sets the processing strategy
  //
  // - forces the template waveform to be recreated
  void setProcessingStrategy(const ProcessingStrategy &strategy);
  // Set the sampling frequency in Hz
  //
  // - may force the template waveform to be recreated
  void setSamplingFrequency(double samplingFrequency);
  // Returns the configured sampling frequency
  double samplingFrequency() const;

  // Returns the number of template waveform samples
  std::size_t size() const;
  // Returns the template waveform duration
  Core::TimeSpan length() const;

  // Returns the actual template waveform starttime which might be different
  // from the starttime configured (due to both sampling rate accuracy and
  // rounding)
  Core::Time startTime() const;
  // Returns the actual template waveform endtime which might be different from
  // the starttime configured (due to both sampling rate accuracy and rounding)
  Core::Time endTime() const;
  // Returns the configured starttime
  Core::Time configuredStartTime() const;
  // Returns the configured endtime
  Core::Time configuredEndTime() const;
  // Sets the template waveform reference time
  void setReferenceTime(const boost::optional<Core::Time> &t = boost::none);
  // Returns the (optional) template waveform reference time
  const boost::optional<Core::Time> &referenceTime() const;

  // Resets the processed template waveform
  //
  // - forces the template waveform to be recreated
  void reset();

 private:
  const GenericRecord &templateWaveform() const;

  static WaveformHandlerIfaceCPtr _waveformHandler;

  ProcessingConfig _processingConfig;

  ProcessingStrategy _processingStrategy{noProcessing};

  // The template waveform reference time
  boost::optional<Core::Time> _referenceTime;

  // The original waveform chunk the `_templateWaveform` is created from
  GenericRecordCPtr _raw;
  // The template waveform (created from `_raw`)
  mutable GenericRecordCPtr _templateWaveform;
};

}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_TEMPLATEWAVEFORM_H_
