#include "rms.h"

#include <seiscomp/core/datetime.h>
#include <seiscomp/core/strings.h>
#include <seiscomp/core/timewindow.h>
#include <seiscomp/datamodel/comment.h>

#include <algorithm>
#include <boost/algorithm/string/join.hpp>
#include <boost/optional/optional.hpp>
#include <cassert>
#include <cmath>
#include <sstream>
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

namespace {
struct TimeInfo {
  Core::Time startTime;
  Core::Time endTime;
  Core::Time referenceTime;

  friend std::istream &operator>>(std::istream &is, TimeInfo &timeInfo) {
    std::string tmp;
    is >> tmp;

    std::vector<std::string> tokens;
    Core::split(tokens, tmp,
                settings::kTemplateWaveformTimeInfoPickCommentIdSep.c_str());
    if (tokens.size() != 3) {
      is.setstate(std::ios::failbit);
      return is;
    }

    auto createTime = [](const std::string &str) {
      auto ret{Core::Time::FromString(str.c_str(), "%FT%T.%fZ")};
      if (!ret) {
        throw ValueException{"invalid time string format: " + str};
      }
      return ret;
    };

    try {
      timeInfo.startTime = createTime(tokens[0]);
      timeInfo.endTime = createTime(tokens[1]);
      timeInfo.referenceTime = createTime(tokens[2]);
    } catch (ValueException &) {
      is.setstate(std::ios::failbit);
    }

    return is;
  }
};
}  // namespace

RMSAmplitude::RMSAmplitude() {
  _type = "Mrms";
  _unit = "M/S";
}

void RMSAmplitude::computeTimeWindow() {
  const auto &env{environment()};
  assert(!env.picks.empty());

  std::vector<Core::Time> pickTimes;
  Core::TimeSpan maxTemplateWaveformStartTimeOffset;
  Core::TimeSpan maxTemplateWaveformEndTimeOffset;
  for (const auto &pick : env.picks) {
    pickTimes.push_back(pick->time().value());
    for (size_t i = 0; i < pick->commentCount(); ++i) {
      const auto *comment{pick->comment(i)};
      if (static_cast<bool>(comment) &&
          comment->id() == settings::kTemplateWaveformTimeInfoPickCommentId) {
        TimeInfo timeInfo;
        std::stringstream ss{comment->text()};
        if (!(ss >> timeInfo)) {
          continue;
        }

        if (!static_cast<bool>(maxTemplateWaveformStartTimeOffset) ||
            timeInfo.referenceTime - timeInfo.startTime >
                maxTemplateWaveformStartTimeOffset) {
          maxTemplateWaveformStartTimeOffset =
              timeInfo.referenceTime - timeInfo.startTime;
        }
        if (!static_cast<bool>(maxTemplateWaveformEndTimeOffset) ||
            timeInfo.endTime - timeInfo.referenceTime >
                maxTemplateWaveformEndTimeOffset) {
          maxTemplateWaveformEndTimeOffset =
              timeInfo.endTime - timeInfo.referenceTime;
        }
      }
    }
  }

  if (!(static_cast<bool>(maxTemplateWaveformEndTimeOffset) &&
        static_cast<bool>(maxTemplateWaveformEndTimeOffset))) {
    throw Processor::BaseException{"failed to determine required time window"};
  }

  auto bounds{std::minmax_element(std::begin(pickTimes), std::end(pickTimes))};
  setTimeWindow(
      Core::TimeWindow{*bounds.first - maxTemplateWaveformStartTimeOffset,
                       *bounds.second + maxTemplateWaveformEndTimeOffset});
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
  for (size_t i = idxRange.begin; i < idxRange.end; ++i) {
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
  const auto &env{environment()};
  assert(!env.picks.empty());

  std::vector<std::string> publicIds;
  for (const auto &pick : env.picks) {
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
    const auto &origin{env.hypocenter};
    assert(origin);
    for (std::size_t i = 0; i < origin->commentCount(); ++i) {
      if (origin->comment(i)->id() == settings::kDetectorIdCommentId) {
        auto comment{util::make_smart<DataModel::Comment>()};
        comment->setId(settings::kDetectorIdCommentId);
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
