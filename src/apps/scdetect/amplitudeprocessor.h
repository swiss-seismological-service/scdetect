#ifndef SCDETECT_APPS_SCDETECT_AMPLITUDEPROCESSOR_H_
#define SCDETECT_APPS_SCDETECT_AMPLITUDEPROCESSOR_H_

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
#include <seiscomp/processing/stream.h>

#include <boost/optional/optional.hpp>
#include <memory>
#include <ostream>
#include <unordered_map>
#include <vector>

#include "timewindowprocessor.h"
#include "waveformoperator.h"

namespace Seiscomp {
namespace detect {

// Base class for amplitude processors
//
// - alternative implementation to `Processing::AmplitudeProcessor`
// - XXX(damb): currently, the concept of a *trigger* is not provided. Instead,
// amplitudes are computed based on the `TimeWindowProcessor`'s time window.
class AmplitudeProcessor : public TimeWindowProcessor {
 public:
  AmplitudeProcessor(const std::string &id);

  struct Config {
    // Defines the beginning of the time window used for amplitude analysis
    // with regard to the beginning of the overall time window
    boost::optional<Core::TimeSpan> signalBegin;
    // Defines the end of the time window used for amplitude analysis with
    // regard to the end of the overall time window
    boost::optional<Core::TimeSpan> signalEnd;
  };

  struct DeconvolutionConfig {
    // Indicates whether deconvolution is enabled `true` or not `false`
    bool enabled{false};
    // Taper length in seconds when deconvolving the data
    double responseTaperLength{5};
    // Defines the end of the left-hand side cosine-taper in Hz applied to the
    // frequency spectrum. I.e. the spectrum is tapered between 0Hz and
    // `minimumResponseTaperFrequency`. A value less than or equal to zero
    // disables tapering.
    double minimumResponseTaperFrequency{0.00833333};  // 120 seconds
    // Defines the beginning of the right-hand side cosine-taper in Hz applied
    // to the frequency spectrum. I.e. the spectrum is tapered between
    // `maximumResponseTaperFrequency` and the Nyquist frequency. A value less
    // than or equal to zero disables tapering.
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
  struct Amplitude : WaveformProcessor::Result {
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

  // Configures the beginning of the time window used for amplitude calculation
  // (with regard to the beginning of the overall time window)
  void setSignalBegin(const boost::optional<Core::TimeSpan> &signalBegin);
  // Returns the beginning of the time window used for amplitude calculation
  Core::Time signalBegin() const;
  // Configures the beginning of the time window used for amplitude calculation
  // (with regard to the end of the overall time window)
  void setSignalEnd(const boost::optional<Core::TimeSpan> &signalEnd);
  // Returns the end of the time window used for amplitude calculation
  Core::Time signalEnd() const;

  // Returns the amplitude type
  const std::string &type() const;
  // Returns the amplitude unit
  const std::string &unit() const;

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
    size_t begin;
    size_t end;
  };

  // Compute the amplitude from `data` in `idxRange`
  virtual void computeAmplitude(const DoubleArray &data,
                                const IndexRange &idxRange,
                                Amplitude &amplitude) = 0;

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
                              const Processing::Stream &streamConfig,
                              const DeconvolutionConfig &deconvolutionConfig,
                              DoubleArray &data);

  // Deconvolve `data` using the sensor response `resp`. Implies both
  // integrating and deriving `data` in order to *convert* `data` into the
  // desired unit.
  //
  // - `numberOfIntegrations` is an integer where a `data` is integrated if
  // greater than zero and derived if less than zero
  virtual bool deconvolveData(StreamState &streamState,
                              Processing::Response *resp,
                              const DeconvolutionConfig &config,
                              int numberOfIntegrations, DoubleArray &data);

  // Derives `data` `numberOfDerivations` times
  virtual bool deriveData(StreamState &streamState, int numberOfDerivations,
                          DoubleArray &data);

  // Amplitude processor configuration
  Config _config;
  // Amplitude processor *environment*
  Environment _environment;

  // The amplitude type
  std::string _type;
  // The amplitude unit
  std::string _unit;
};

/* ------------------------------------------------------------------------- */
// Base class for reducing amplitude processors
//
// - handles multiple streams which are reduced to a single amplitude
// - TODO(damb): implement SNR facilities
class ReducingAmplitudeProcessor : public AmplitudeProcessor {
 public:
  ReducingAmplitudeProcessor(const std::string &id);

  // Sets the `filter` for all registered streams
  //
  // - configuring the `filter` can be done only before the first record was
  // fed or after resetting the processor
  void setFilter(Filter *filter, const Core::TimeSpan &initTime) override;

  bool feed(const Record *record) override;

  void reset() override;

  // Registers a new stream with `streamConfig`
  //
  // - adding additional streams can be done only before the first record was
  // fed
  virtual void add(const std::string &netCode, const std::string &staCode,
                   const std::string &locCode,
                   const Processing::Stream &streamConfig);

  // Returns a the registered waveform stream identifiers
  std::vector<std::string> waveformStreamIds() const;

  // Dump the buffered data to `out`
  void dumpBufferedData(std::ostream &out);

 protected:
  // Reduce `data` regarding an amplitude calculation where `noiseInfos`
  // corresponds the individual noise offset.
  //
  // - returns the reduced result
  virtual DoubleArrayCPtr reduceAmplitudeData(
      const std::vector<DoubleArray const *> &data,
      const std::vector<NoiseInfo> &noiseInfos, const IndexRange &idxRange) = 0;

  // Compute an overall signal-to-noise ratio
  virtual boost::optional<double> reduceNoiseData(
      const std::vector<DoubleArray const *> &data,
      const std::vector<IndexRange> &idxRanges,
      const std::vector<NoiseInfo> &noiseInfos);

  StreamState &streamState(const Record *record) override;

  void process(StreamState &streamState, const Record *record,
               const DoubleArray &filteredData) override;

  bool store(const Record *record) override;

  bool fill(detect::StreamState &streamState, const Record *record,
            DoubleArrayPtr &data) override;

  bool processIfEnoughDataReceived(StreamState &streamState,
                                   const Record *record,
                                   const DoubleArray &filteredData) override;

  bool enoughDataReceived(const StreamState &streamState) const override;

  using Buffer = DoubleArray;

  struct StreamItem {
    // Stream configuration including sensor response etc.
    Processing::Stream streamConfig;
    // Current stream state
    WaveformProcessor::StreamState streamState;
    // Time window buffer
    Buffer buffer;

    DeconvolutionConfig deconvolutionConfig;

    // Defines the needed samples (including both `_initTime` (i.e. used for
    // filter initialization) and the number of samples needed to enable
    // noise/amplitude analysis.
    size_t neededSamples{0};

    // stream specific noise offset
    boost::optional<double> noiseOffset;
  };

  using WaveformStreamId = std::string;
  using StreamMap = std::unordered_map<WaveformStreamId, StreamItem>;
  StreamMap _streams;

 private:
  // Returns if streams may be added to the processor
  bool locked() const;
  // The common sampling frequency
  boost::optional<double> _commonSamplingFrequency;

  // Pointer to the configured filter
  std::unique_ptr<Filter> _filter;
};

}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_AMPLITUDEPROCESSOR_H_
