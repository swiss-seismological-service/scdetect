#ifndef SCDETECT_APPS_CC_AMPLITUDEPROCESSOR_H_
#define SCDETECT_APPS_CC_AMPLITUDEPROCESSOR_H_

#include <seiscomp/core/baseobject.h>
#include <seiscomp/core/datetime.h>
#include <seiscomp/core/defs.h>
#include <seiscomp/core/record.h>
#include <seiscomp/core/timewindow.h>
#include <seiscomp/core/typedarray.h>
#include <seiscomp/datamodel/amplitude.h>
#include <seiscomp/datamodel/origin.h>
#include <seiscomp/datamodel/pick.h>
#include <seiscomp/datamodel/sensorlocation.h>
#include <seiscomp/processing/response.h>

#include <boost/optional/optional.hpp>
#include <memory>
#include <vector>

#include "amplitude/factory.h"
#include "processing/stream_config.h"
#include "processing/timewindow_processor.h"

namespace Seiscomp {
namespace detect {

// Base class for amplitude processors
//
// - alternative implementation to `Processing::AmplitudeProcessor`
// - XXX(damb): currently, the concept of a *trigger* is not provided. Instead,
// amplitudes are computed based on the `TimeWindowProcessor`'s time window.
class AmplitudeProcessor : public processing::TimeWindowProcessor {
 public:
  using Factory = amplitude::Factory;

  class BaseException : public Processor::BaseException {
   public:
    using Processor::BaseException::BaseException;
    BaseException();
  };

  struct Config {
    // Defines the beginning of the time window used for amplitude analysis
    // with regard to the beginning of the overall time window
    boost::optional<Core::TimeSpan> signalBegin;
    // Defines the end of the time window used for amplitude analysis with
    // regard to the end of the overall time window
    boost::optional<Core::TimeSpan> signalEnd;
  };

  struct DeconvolutionConfig {
    DeconvolutionConfig() = default;
    explicit DeconvolutionConfig(
        const binding::StreamConfig::DeconvolutionConfig &config);
    // Indicates whether deconvolution is enabled `true` or whether not `false`
    bool enabled{false};
    // Taper length in seconds when deconvolving the data
    double responseTaperLength{5};
    // Defines the end of the left-hand side cosine-taper in Hz applied to the
    // frequency spectrum. I.e. the spectrum is tapered between 0Hz and
    // `minimumResponseTaperFrequency`. A value less than or equal to zero
    // disables left-hand side tapering.
    double minimumResponseTaperFrequency{0.00833333};  // 120 seconds
    // Defines the beginning of the right-hand side cosine-taper in Hz applied
    // to the frequency spectrum. I.e. the spectrum is tapered between
    // `maximumResponseTaperFrequency` and the Nyquist frequency. A value less
    // than or equal to zero disables right-hand side tapering.
    double maximumResponseTaperFrequency{0};
  };

  struct AmplitudeValue {
    double value;
    boost::optional<double> lowerUncertainty;
    boost::optional<double> upperUncertainty;
  };

  // Amplitude time / time window
  struct AmplitudeTime {
    // The amplitude reference time
    Core::Time reference;
    // Duration in seconds before `reference`. Must be positive.
    double begin{0};
    // Duration in seconds after `reference`. Must be positive.
    double end{0};
  };

  struct Environment {
    DataModel::OriginCPtr hypocenter;
    DataModel::SensorLocationCPtr receiver;
    std::vector<DataModel::PickCPtr> picks;
  };

  DEFINE_SMARTPOINTER(Amplitude);
  struct Amplitude : Core::BaseObject {
    AmplitudeValue value;
    AmplitudeTime time;

    using WaveformStreamIds = std::vector<std::string>;
    // Waveform stream identifiers the amplitude is referencing
    boost::optional<WaveformStreamIds> waveformStreamIds;
    // Dominant period in samples (NOT seconds) w.r.t. the time window the
    // amplitude was measured; not used for so-called *duration magnitudes*
    boost::optional<double> dominantPeriod;
    // The signal-to-noise ratio w.r.t. the time window the amplitude was
    // measured
    boost::optional<double> snr;
  };
  using PublishAmplitudeCallback = std::function<void(
      const AmplitudeProcessor *, const Record *, AmplitudeCPtr)>;

  // Sets the `callback` in order to publish detections
  void setResultCallback(const PublishAmplitudeCallback &callback);

  // Configures the beginning of the time window used for amplitude calculation
  // (with regard to the beginning of the overall time window)
  void setSignalBegin(const boost::optional<Core::TimeSpan> &signalBegin);
  // Returns the beginning of the time window used for amplitude calculation
  Core::Time signalBegin() const;
  // Configures the end of the time window used for amplitude calculation
  // (with regard to the end of the overall time window)
  void setSignalEnd(const boost::optional<Core::TimeSpan> &signalEnd);
  // Returns the end of the time window used for amplitude calculation
  Core::Time signalEnd() const;

  // Returns the amplitude type
  const std::string &type() const;
  // Returns the amplitude unit
  const std::string &unit() const;

  // Returns the waveform stream identifiers the amplitude processor is
  // associated with
  virtual std::vector<std::string> associatedWaveformStreamIds() const;

  // Sets the *environment* of the amplitude processor
  virtual void setEnvironment(const DataModel::OriginCPtr &hypocenter,
                              const DataModel::SensorLocationCPtr &receiver,
                              const std::vector<DataModel::PickCPtr> &picks);
  // Returns the `AmplitudeProcessor`'s environment
  const Environment &environment() const;

  // Allows to finalize the `amplitude` which was previously created by client
  // code
  virtual void finalize(DataModel::Amplitude *amplitude) const;

 protected:
  struct NoiseInfo {
    // The noise offset
    double offset{0};
    // The noise amplitude
    double amplitude{0};
  };

  struct IndexRange {
    std::size_t begin;
    std::size_t end;
  };

  using Buffer = DoubleArray;
  using Response = Processing::Response;

  // Sets the type of amplitudes the amplitude processor is producing
  void setType(std::string type);
  // Sets the unit of amplitudes the amplitude processor is producing
  void setUnit(std::string unit);

  // Compute the noise from `data` in the window defined by `idxRange`. While
  // `noiseOffset` refers to an offset applied when computing the noise the
  // `noiseAmplitude` refers to the noise amplitude computed.
  //
  // - The default implementation returns the median of `data` (sliced regarding
  // `idxRange`) as `noiseOffset` and twice the rms regarding `noiseOffset` as
  // `noiseAmplitude`
  virtual bool computeNoise(const DoubleArray &data, const IndexRange &idxRange,
                            NoiseInfo &NoiseInfo);

  // Preprocess `data`
  //
  // - called just before the noise and amplitude calculation is performed
  // - the default implementation does nothing
  virtual void preprocessData(StreamState &streamState,
                              const processing::StreamConfig &streamConfig,
                              const DeconvolutionConfig &deconvolutionConfig,
                              DoubleArray &data);

  // Deconvolve `data` using the sensor response `resp`. Implies both
  // integrating and deriving `data` in order to *convert* `data` into the
  // desired unit.
  //
  // - `numberOfIntegrations` is an integer where a `data` is integrated if
  // greater than zero and derived if less than zero
  virtual bool deconvolveData(StreamState &streamState, Response *resp,
                              const DeconvolutionConfig &config,
                              int numberOfIntegrations, DoubleArray &data);

  // Derives `data` `numberOfDerivations` times
  virtual bool deriveData(StreamState &streamState, int numberOfDerivations,
                          DoubleArray &data);

  void emitAmplitude(const Record *record, const AmplitudeCPtr &amplitude);

  // Amplitude processor configuration
  Config _config;

 private:
  // Amplitude processor *environment*
  Environment _environment;

  // The amplitude type
  std::string _type;
  // The amplitude unit
  std::string _unit;

  // The callback invoked when there is an amplitude to publish
  PublishAmplitudeCallback _resultCallback;
};

}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_CC_AMPLITUDEPROCESSOR_H_
