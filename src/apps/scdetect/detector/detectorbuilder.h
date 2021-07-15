#ifndef SCDETECT_APPS_SCDETECT_DETECTOR_DETECTORBUILDER_H_
#define SCDETECT_APPS_SCDETECT_DETECTOR_DETECTORBUILDER_H_

#include <seiscomp/datamodel/arrival.h>
#include <seiscomp/datamodel/pick.h>
#include <seiscomp/datamodel/sensorlocation.h>

#include <boost/filesystem.hpp>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "../builder.h"
#include "../config.h"
#include "../waveform.h"
#include "linker/strategy.h"
#include "pot.h"
#include "templatewaveformprocessor.h"

namespace Seiscomp {
namespace detect {
namespace detector {

class DetectorWaveformProcessor;

class DetectorBuilder : public Builder<DetectorWaveformProcessor> {
 public:
  DetectorBuilder(const std::string &id, const std::string &originId);

  DetectorBuilder &setConfig(const DetectorConfig &config, bool playback);

  DetectorBuilder &setEventParameters();
  // Set stream related template configuration where `streamId` refers to the
  // waveform stream identifier of the stream to be processed.
  DetectorBuilder &setStream(const std::string &streamId,
                             const StreamConfig &streamConfig,
                             WaveformHandlerIfacePtr &wfHandler,
                             const boost::filesystem::path &pathDebugInfo = "");
  // Set the path to the debug info directory
  DetectorBuilder &setDebugInfoDir(const boost::filesystem::path &path);

 protected:
  void finalize() override;

  bool isValidArrival(const DataModel::ArrivalCPtr arrival,
                      const DataModel::PickCPtr pick);

 private:
  struct TemplateProcessorConfig {
    // Template matching processor
    std::unique_ptr<TemplateWaveformProcessor> processor;

    struct MetaData {
      // The template's sensor location associated
      DataModel::SensorLocationCPtr sensorLocation;
      // The template related pick
      DataModel::PickCPtr pick;
      // The template related arrival
      DataModel::ArrivalCPtr arrival;
    } metadata;
  };

  std::string _originId;

  std::vector<POT::ArrivalPick> _arrivalPicks;

  using TemplateProcessorConfigs =
      std::unordered_map<std::string, TemplateProcessorConfig>;
  TemplateProcessorConfigs _processorConfigs;

  static const std::unordered_map<std::string, linker::MergingStrategy::Type>
      _mergingStrategyLookupTable;
};

}  // namespace detector
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_DETECTOR_DETECTORBUILDER_H_
