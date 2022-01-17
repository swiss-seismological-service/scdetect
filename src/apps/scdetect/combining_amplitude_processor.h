#ifndef SCDETECT_APPS_SCDETECT_COMBININGAMPLITUDEPROCESSOR_H_
#define SCDETECT_APPS_SCDETECT_COMBININGAMPLITUDEPROCESSOR_H_

#include <boost/optional/optional.hpp>
#include <functional>
#include <iterator>
#include <memory>
#include <unordered_map>
#include <vector>

#include "amplitude_processor.h"
#include "processing/waveform_processor.h"

namespace Seiscomp {
namespace detect {

// A proxy which combines results of the underlying amplitude processors
//
// - forwards data to the underlying amplitude processors
class CombiningAmplitudeProcessor : public detect::AmplitudeProcessor {
 public:
  using CombiningStrategy =
      std::function<void(const std::vector<AmplitudeCPtr> &, Amplitude &)>;

  struct AmplitudeProcessor {
    std::vector<std::string> waveformStreamIds;
    std::unique_ptr<detect::AmplitudeProcessor> processor;
  };

  CombiningAmplitudeProcessor(std::vector<AmplitudeProcessor> combined,
                              CombiningStrategy strategy);

  void reset() override;

  void close() const override;

  void computeTimeWindow() override;

  std::vector<std::string> associatedWaveformStreamIds() const override;

 protected:
  processing::WaveformProcessor::StreamState *streamState(
      const Record *record) override;

  bool store(const Record *record) override;

  struct UnderlyingProcessor {
    std::unique_ptr<detect::AmplitudeProcessor> amplitudeProcessor;
    detect::AmplitudeProcessor::AmplitudeCPtr result;

    void reset();
  };

  using ProcessorId = std::string;
  using Underlying = std::unordered_map<ProcessorId, UnderlyingProcessor>;
  using UnderlyingConstIterator = Underlying::const_iterator;

  class ConstIterator : public UnderlyingConstIterator {
   public:
    ConstIterator() = default;
    explicit ConstIterator(UnderlyingConstIterator it)
        : UnderlyingConstIterator{it} {}

    const detect::AmplitudeProcessor *operator->() {
      return UnderlyingConstIterator::operator->()
          ->second.amplitudeProcessor.get();
    }
    const detect::AmplitudeProcessor &operator*() {
      return *UnderlyingConstIterator::operator*().second.amplitudeProcessor;
    }
  };

  ConstIterator begin() const { return ConstIterator{_underlying.cbegin()}; }
  ConstIterator end() const { return ConstIterator{_underlying.cend()}; }

  const detect::AmplitudeProcessor *underlying(
      const std::string &processorId) const;

 private:
  void computeAmplitude(
      const std::vector<detect::AmplitudeProcessor::AmplitudeCPtr> &amplitudes,
      detect::AmplitudeProcessor::Amplitude &result) const;

  bool allUnderlyingFinished() const;

  void storeCallback(const detect::AmplitudeProcessor *processor,
                     const Record *record,
                     detect::AmplitudeProcessor::AmplitudeCPtr amplitude);

  template <typename T>
  void traverse(T func) {
    for (auto &u : _underlying) {
      func(u.second);
    }
  }

  template <typename T>
  void traverse(T func) const {
    for (const auto &u : _underlying) {
      func(u.second);
    }
  }

  Underlying _underlying;

  using WaveformStreamId = std::string;
  using UnderlyingIdx = std::unordered_multimap<WaveformStreamId, ProcessorId>;
  UnderlyingIdx _underlyingIdx;

  CombiningStrategy _combiningStrategy;
};

}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_COMBININGAMPLITUDEPROCESSOR_H_
