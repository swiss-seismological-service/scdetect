#include "combining_amplitude_processor.h"

#include <seiscomp/core/datetime.h>
#include <seiscomp/core/timewindow.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <memory>
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
    std::vector<AmplitudeProcessor> &&combined, CombiningStrategy strategy)
    : _combiningStrategy{std::move(strategy)} {
  for (auto &proc : combined) {
    std::shared_ptr<detect::AmplitudeProcessor> amplitudeProcessor{
        std::move(proc.processor)};

    amplitudeProcessor->setResultCallback(
        [this](const detect::AmplitudeProcessor *processor,
               const Record *record, AmplitudeCPtr amplitude) {
          storeCallback(processor, record, std::move(amplitude));
        });

    for (const auto &waveformStreamId : proc.waveformStreamIds) {
      _underlying.emplace(waveformStreamId, amplitudeProcessor);
    }

    _underlyingResults[amplitudeProcessor->id()];
  }
}

void CombiningAmplitudeProcessor::reset() {
  detect::AmplitudeProcessor::reset();

  traverse([](decltype(_underlying)::mapped_type &p) { p->reset(); });
  for (auto &u : _underlyingResults) {
    u.second.reset();
  }
}

void CombiningAmplitudeProcessor::computeTimeWindow() {
  Core::Time start;
  Core::Time end;
  for (auto &u : _underlying) {
    u.second->computeTimeWindow();
    const auto tw{u.second->safetyTimeWindow()};

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

bool CombiningAmplitudeProcessor::store(const Record *record) {
  if (allUnderlyingFinished() || finished()) {
    return false;
  }

  auto range{_underlying.equal_range(record->streamID())};
  // dispatch record to underlying amplitude processors
  for (auto i{range.first}; i != range.second; ++i) {
    auto &processor{i->second};
    if (processor->finished() || !processor->enabled()) {
      continue;
    }

    try {
      if (!processor->feed(record)) {
        throw BaseException{"failed to feed data"};
      }
    } catch (BaseException &e) {
      setStatus(processor->status(), processor->statusValue());
      SCDETECT_LOG_ERROR_TAGGED(
          processor->id(),
          "%s: failed to feed data (tw.start=%s, "
          "tw.end=%s) to processor. Reason: status=%d, "
          "statusValue=%f (%s)",
          record->streamID().c_str(), record->startTime().iso().c_str(),
          record->endTime().iso().c_str(), util::asInteger(processor->status()),
          processor->statusValue(), e.what());
      return false;
    }
  }

  if (!allUnderlyingFinished()) {
    return true;
  }

  std::vector<detect::AmplitudeProcessor::AmplitudeCPtr> amplitudes;
  for (auto &resultPair : _underlyingResults) {
    if (resultPair.second) {
      amplitudes.emplace_back(std::move(resultPair.second));
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

void CombiningAmplitudeProcessor::close() const {
  traverse([](const decltype(_underlying)::mapped_type &p) { p->close(); });
}

std::vector<std::string> CombiningAmplitudeProcessor::waveformStreamIds()
    const {
  std::unordered_set<WaveformStreamId> unique;
  for (const auto &u : _underlying) {
    unique.emplace(u.first);
  }

  return std::vector<WaveformStreamId>{std::begin(unique), std::end(unique)};
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
                       return u.second->finished();
                     });
}

void CombiningAmplitudeProcessor::storeCallback(
    const detect::AmplitudeProcessor *processor,
    const Record *record,  // NOLINT(misc-unused-parameters)
    detect::AmplitudeProcessor::AmplitudeCPtr amplitude) {
  try {
    _underlyingResults.at(processor->id()) = std::move(amplitude);
  } catch (std::out_of_range &) {
  }
}

}  // namespace detect
}  // namespace Seiscomp
