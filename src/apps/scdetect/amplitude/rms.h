#ifndef SCDETECT_APPS_SCDETECT_AMPLITUDE_RMS_H_
#define SCDETECT_APPS_SCDETECT_AMPLITUDE_RMS_H_

#include "../amplitudeprocessor.h"

namespace Seiscomp {
namespace detect {
namespace amplitude {

// Computes the RMS amplitude
//
// - the amplitude is computed on velocity seismograms
class RMSAmplitude : public ReducingAmplitudeProcessor {
 public:
  RMSAmplitude(const std::string &id);

  enum class SignalUnit {
    // displacement
    kMeter = -1,
    // velocity
    kMeterPerSeconds,
    // acceleration
    kMeterPerSecondsSquared,
  };

  // Computes time time window based on environment picks
  void computeTimeWindow() override;

 protected:
  // Preprocess `data` by means of deconvolution
  void preprocessData(StreamState &streamState, Processing::Sensor *sensor,
                      const DeconvolutionConfig &deconvolutionConfig,
                      DoubleArray &data) override;

  DoubleArrayCPtr reduceAmplitudeData(
      const std::vector<DoubleArray const *> &data,
      const std::vector<NoiseInfo> &noiseInfos,
      const IndexRange &idxRange) override;

  boost::optional<double> reduceNoiseData(
      const std::vector<DoubleArray const *> &data,
      const std::vector<IndexRange> &idxRanges,
      const std::vector<NoiseInfo> &noiseInfos) override;

  void computeAmplitude(const DoubleArray &data, const IndexRange &idxRange,
                        Amplitude &amplitude) override;

  // Finalizes `amplitude`
  //
  // - attaches both pick and stream related identifiers by means of
  // `DataModel::Comment`s
  void finalize(DataModel::Amplitude *amplitude) const override;
};

RMSAmplitude::SignalUnit signalUnitFromString(const std::string &signalUnit);

}  // namespace amplitude
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_AMPLITUDE_RMS_H_
