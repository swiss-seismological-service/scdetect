#ifndef SCDETECT_APPS_SCDETECT_REDUCINGAMPLITUDEPROCESSOR_H_
#define SCDETECT_APPS_SCDETECT_REDUCINGAMPLITUDEPROCESSOR_H_

#include <seiscomp/core/datetime.h>
#include <seiscomp/core/typedarray.h>
#include <seiscomp/processing/stream.h>

#include <boost/optional/optional.hpp>
#include <memory>
#include <ostream>
#include <string>
#include <unordered_map>
#include <vector>

#include "amplitude_processor.h"
#include "processing/waveform_processor.h"

namespace Seiscomp {
namespace detect {

// Abstract base class for reducing amplitude processors
//
// - handles multiple streams which are reduced to a single amplitude
// - TODO(damb): implement SNR facilities
class ReducingAmplitudeProcessor : public AmplitudeProcessor {
 public:
  // Sets the common filter `filter` for all registered streams
  //
  // - configuring the `filter` can be done only before the first record was
  // fed or after resetting the processor
  void setFilter(std::unique_ptr<Filter> &&filter,
                 const Core::TimeSpan &initTime);

  bool feed(const Record *record) override;

  void reset() override;

  // Registers a new stream with `streamConfig` and `deconvolutionConfig`
  //
  // - adding additional streams can be done only before the first record was
  // fed
  virtual void add(
      const std::string &netCode, const std::string &staCode,
      const std::string &locCode, const Processing::Stream &streamConfig,
      const AmplitudeProcessor::DeconvolutionConfig &deconvolutionConfig);

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

  // Compute the amplitude from `data` in `idxRange`
  virtual void computeAmplitude(const DoubleArray &data,
                                const IndexRange &idxRange,
                                Amplitude &amplitude) = 0;

  // Compute an overall signal-to-noise ratio
  virtual boost::optional<double> reduceNoiseData(
      const std::vector<DoubleArray const *> &data,
      const std::vector<IndexRange> &idxRanges,
      const std::vector<NoiseInfo> &noiseInfos);

  processing::WaveformProcessor::StreamState *streamState(
      const Record *record) override;

  void process(StreamState &streamState, const Record *record,
               const DoubleArray &filteredData) override;

  bool store(const Record *record) override;

  bool fill(processing::StreamState &streamState, const Record *record,
            DoubleArrayPtr &data) override;

  bool processIfEnoughDataReceived(StreamState &streamState,
                                   const Record *record,
                                   const DoubleArray &filteredData) override;

  bool enoughDataReceived(const StreamState &streamState) const override;

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

#endif  // SCDETECT_APPS_SCDETECT_REDUCINGAMPLITUDEPROCESSOR_H_
