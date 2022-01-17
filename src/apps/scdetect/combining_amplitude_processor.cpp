#include "combining_amplitude_processor.h"

#include <seiscomp/core/datetime.h>
#include <seiscomp/core/timewindow.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "amplitude_processor.h"
#include "log.h"
#include "processing/processor.h"
#include "processing/waveform_processor.h"
#include "util/memory.h"
#include "util/util.h"

namespace Seiscomp {
namespace detect {

CombiningAmplitudeProcessor::CombiningAmplitudeProcessor(
    std::vector<AmplitudeProcessor> combined, CombiningStrategy strategy)
    : _combiningStrategy{std::move(strategy)} {
  for (auto &proc : combined) {
    proc.processor->setResultCallback(
        [this](const detect::AmplitudeProcessor *processor,
               const Record *record, AmplitudeCPtr amplitude) {
          storeCallback(processor, record, std::move(amplitude));
        });

    for (const auto &waveformStreamId : proc.waveformStreamIds) {
      _underlyingIdx.emplace(waveformStreamId, proc.processor->id());
    }

    auto processorId{proc.processor->id()};
    _underlying.emplace(processorId,
                        UnderlyingProcessor{std::move(proc.processor)});
  }
}

void CombiningAmplitudeProcessor::reset() {
  detect::AmplitudeProcessor::reset();
  traverse([](decltype(_underlying)::mapped_type &p) { p.reset(); });
}

void CombiningAmplitudeProcessor::close() const {
  traverse([](const decltype(_underlying)::mapped_type &p) {
    p.amplitudeProcessor->close();
  });
}

void CombiningAmplitudeProcessor::computeTimeWindow() {
  Core::Time start;
  Core::Time end;
  for (auto &u : _underlying) {
    u.second.amplitudeProcessor->computeTimeWindow();
    const auto tw{u.second.amplitudeProcessor->safetyTimeWindow()};

    if (!start || tw.startTime() < start) {
      start = tw.startTime();
    }
    if (!end || tw.endTime() > end) {
      end = tw.endTime();
    }
  }

  assert((start && end));
  assert((start < end));
  setTimeWindow(Core::TimeWindow{start, end});
}

std::vector<std::string>
CombiningAmplitudeProcessor::associatedWaveformStreamIds() const {
  std::unordered_set<WaveformStreamId> unique;
  for (const auto &u : _underlyingIdx) {
    unique.emplace(u.first);
  }

  return std::vector<WaveformStreamId>{std::begin(unique), std::end(unique)};
}

bool CombiningAmplitudeProcessor::store(const Record *record) {
  if (allUnderlyingFinished() || finished()) {
    return false;
  }

  auto range{_underlyingIdx.equal_range(record->streamID())};
  // dispatch record to underlying amplitude processors
  for (auto i{range.first}; i != range.second; ++i) {
    const auto &processorId{i->second};
    auto &processor{_underlying.at(processorId).amplitudeProcessor};
    if (processor->finished() || !processor->enabled()) {
      continue;
    }

    try {
      if (!processor->feed(record) && processor->finished()) {
        throw BaseException{"failed to feed data"};
      }
    } catch (BaseException &e) {
      setStatus(processor->status(), processor->statusValue());

      logging::TaggedMessage msg{
          record->streamID(),
          "Failed to feed data (tw.start=" + record->startTime().iso() +
              ", tw.end=" + record->endTime().iso() + "). Reason: status=" +
              std::to_string(util::asInteger(processor->status())) +
              ", status_value=" + std::to_string(processor->statusValue()) +
              " (" + e.what() + ")"};

      SCDETECT_LOG_ERROR_TAGGED(processor->id(), "%s",
                                logging::to_string(msg).c_str());
      return false;
    }
  }

  if (!allUnderlyingFinished()) {
    return true;
  }

  std::vector<detect::AmplitudeProcessor::AmplitudeCPtr> amplitudes;
  for (auto &u : _underlying) {
    if (u.second.result) {
      amplitudes.emplace_back(std::move(u.second.result));
    }
  }
  if (amplitudes.empty()) {
    setStatus(Status::kError, 0);
    return false;
  }

  auto result{util::make_smart<detect::AmplitudeProcessor::Amplitude>()};
  try {
    computeAmplitude(amplitudes, *result);
  } catch (processing::WaveformProcessor::BaseException &e) {
    setStatus(Status::kError, 0);
    return false;
  }

  setStatus(Status::kFinished, 100);
  emitAmplitude(record, result);

  return true;
}

const detect::AmplitudeProcessor *CombiningAmplitudeProcessor::underlying(
    const std::string &processorId) const {
  return _underlying.at(processorId).amplitudeProcessor.get();
}

processing::WaveformProcessor::StreamState *
CombiningAmplitudeProcessor::streamState(
    const Record *record)  // NOLINT(misc-unused-parameters)
{
  return nullptr;
}

void CombiningAmplitudeProcessor::computeAmplitude(
    const std::vector<detect::AmplitudeProcessor::AmplitudeCPtr> &amplitudes,
    detect::AmplitudeProcessor::Amplitude &result) const {
  assert(_combiningStrategy);
  _combiningStrategy(amplitudes, result);
}

bool CombiningAmplitudeProcessor::allUnderlyingFinished() const {
  return std::all_of(std::begin(_underlying), std::end(_underlying),
                     [](const decltype(_underlying)::value_type &u) {
                       return u.second.amplitudeProcessor->finished();
                     });
}

void CombiningAmplitudeProcessor::storeCallback(
    const detect::AmplitudeProcessor *processor,
    const Record *record,  // NOLINT(misc-unused-parameters)
    detect::AmplitudeProcessor::AmplitudeCPtr amplitude) {
  try {
    _underlying.at(processor->id()).result = std::move(amplitude);
  } catch (std::out_of_range &) {
  }
}

void CombiningAmplitudeProcessor::UnderlyingProcessor::reset() {
  amplitudeProcessor->reset();
  result.reset();
}

}  // namespace detect
}  // namespace Seiscomp
