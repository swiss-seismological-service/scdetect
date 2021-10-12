#include "rms.h"

#include <seiscomp/core/datetime.h>
#include <seiscomp/core/timewindow.h>
#include <seiscomp/datamodel/comment.h>

#include <algorithm>
#include <boost/algorithm/string/join.hpp>
#include <boost/optional/optional.hpp>
#include <cmath>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "../settings.h"
#include "../util/math.h"
#include "../util/memory.h"
#include "../util/util.h"

namespace Seiscomp {
namespace detect {
namespace amplitude {

RMSAmplitude::RMSAmplitude(const std::string &id)
    : ReducingAmplitudeProcessor{id} {
  _type = "Mrms";
  _unit = "M/S";
}

void RMSAmplitude::computeTimeWindow() {
  if (_environment.picks.empty()) {
    throw Processor::BaseException(
        "failed to compute time window: missing picks");
  }

  std::vector<Core::Time> pickTimes;
  Core::TimeSpan maxTemplateWaveformDuration;
  for (const auto &pick : _environment.picks) {
    pickTimes.push_back(pick->time().value());
    for (size_t i = 0; i < pick->commentCount(); ++i) {
      const auto comment{pick->comment(i)};
      if (comment &&
          comment->id() == settings::kTemplateWaveformDurationPickCommentId) {
        try {
          auto converted{std::stod(comment->text())};
          if (converted > maxTemplateWaveformDuration) {
            maxTemplateWaveformDuration = converted;
          }
        } catch (std::invalid_argument &) {
          continue;
        } catch (std::out_of_range &) {
          continue;
        }
      }
    }
  }

  if (!maxTemplateWaveformDuration) {
    throw Processor::BaseException{"failed to determine required time window"};
  }

  auto bounds{std::minmax_element(std::begin(pickTimes), std::end(pickTimes))};
  // XXX(damb): Use twice the template waveform duration as time window for
  // amplitude calculation plus the time span included by all available picks.
  setTimeWindow(Core::TimeWindow{*bounds.first - maxTemplateWaveformDuration,
                                 *bounds.second + maxTemplateWaveformDuration});
}

void RMSAmplitude::preprocessData(
    StreamState &streamState, const Processing::Stream &streamConfig,
    const DeconvolutionConfig &deconvolutionConfig, DoubleArray &data) {
  auto sensor{streamConfig.sensor()};
  if (!sensor || !sensor->response()) {
    setStatus(Status::kMissingResponse, 0);
    return;
  }

  if (streamConfig.gain == 0) {
    setStatus(Status::kMissingGain, -1);
    return;
  }

  SignalUnit signalUnit;
  try {
    signalUnit = signalUnitFromString(sensor->unit());
  } catch (std::out_of_range &) {
    setStatus(Status::kIncompatibleUnit, 0);
    return;
  }

  const auto numberOfIntegrations{util::asInteger(signalUnit)};
  if (deconvolutionConfig.enabled) {
    if (!deconvolveData(streamState, sensor->response(), deconvolutionConfig,
                        numberOfIntegrations, data)) {
      setStatus(Status::kDeconvolutionFailed, 0);
      return;
    }
  } else {
    if (numberOfIntegrations > 0) {
      // currently, integration is not supported without deconvolution
      setStatus(Status::kError, 0);
      return;
    } else if (numberOfIntegrations < 0) {
      deriveData(streamState, std::abs(numberOfIntegrations), data);
    }
  }

  // XXX(damb): `streamConfig` is not modified
  const_cast<Processing::Stream &>(streamConfig).applyGain(data);
}

DoubleArrayCPtr RMSAmplitude::reduceAmplitudeData(
    const std::vector<DoubleArray const *> &data,
    const std::vector<NoiseInfo> &noiseInfos, const IndexRange &idxRange) {
  if (data.size() != noiseInfos.size()) {
    return nullptr;
  }

  const auto numberOfStreams{data.size()};

  std::vector<double> samples;
  for (size_t i = idxRange.begin; i <= idxRange.end; ++i) {
    double rms{0};
    for (size_t j = 0; j < numberOfStreams; ++j) {
      rms += util::square(data[j]->get(i) - noiseInfos[j].offset);
    }
    samples.push_back(sqrt(rms));
  }

  return util::make_smart<DoubleArray>(static_cast<int>(samples.size()),
                                       samples.data());
}

boost::optional<double> RMSAmplitude::reduceNoiseData(
    const std::vector<DoubleArray const *> &data,
    const std::vector<IndexRange> &idxRanges,
    const std::vector<NoiseInfo> &noiseInfos) {
  // TODO(damb): to be implemented
  return boost::none;
}

void RMSAmplitude::computeAmplitude(const DoubleArray &data,
                                    const IndexRange &idxRange,
                                    AmplitudeProcessor::Amplitude &amplitude) {
  auto rangeBegin{data.begin() + idxRange.begin};
  auto rangeEnd{data.begin() + idxRange.end};

  // XXX(damb): currently, the amplitude is computed as the max element-wise
  // RMS
  auto it{std::max_element(rangeBegin, rangeEnd)};
  if (it == data.end()) {
    setStatus(Status::kError, 0);
    return;
  }
  amplitude.value.value = *it;

  // TODO(damb): compute SNR
}

void RMSAmplitude::finalize(DataModel::Amplitude *amplitude) const {
  std::vector<std::string> publicIds;
  for (const auto &pick : _environment.picks) {
    publicIds.push_back(pick->publicID());
  }

  // pick public identifiers
  {
    auto comment{util::make_smart<DataModel::Comment>()};
    comment->setId("scdetectRMSAmplitudePicks");
    comment->setText(boost::algorithm::join(publicIds, settings::kPublicIdSep));
    amplitude->add(comment.get());
  }

  // waveform stream identifiers
  {
    auto comment{util::make_smart<DataModel::Comment>()};
    comment->setId("scdetectRMSAmplitudeStreams");
    comment->setText(boost::algorithm::join(util::map_keys(_streams),
                                            settings::kWaveformStreamIdSep));
    amplitude->add(comment.get());
  }

  // forward reference of the detector which declared the origin
  {
    const auto &origin{_environment.hypocenter};
    for (std::size_t i = 0; i < origin->commentCount(); ++i) {
      if (origin->comment(i)->id() == "scdetectDetectorId") {
        auto comment{util::make_smart<DataModel::Comment>()};
        comment->setId("scdetectDetectorId");
        comment->setText(origin->comment(i)->text());
        amplitude->add(comment.get());
        break;
      }
    }
  }
}

/* ------------------------------------------------------------------------- */
RMSAmplitude::SignalUnit signalUnitFromString(const std::string &signalUnit) {
  using SignalUnit = RMSAmplitude::SignalUnit;
  // SEED names: see SEEDManual, ch.5 / 34, p.47
  static const std::unordered_map<std::string, SignalUnit> lookUpTable{
      {"M", SignalUnit::kMeter},
      {"M/S", SignalUnit::kMeterPerSeconds},
      {"M/S**2", SignalUnit::kMeterPerSecondsSquared}};

  return lookUpTable.at(signalUnit);
}

}  // namespace amplitude
}  // namespace detect
}  // namespace Seiscomp
