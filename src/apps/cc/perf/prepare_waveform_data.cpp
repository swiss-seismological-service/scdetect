#include <seiscomp/core/datetime.h>
#include <seiscomp/core/genericrecord.h>
#include <seiscomp/core/timewindow.h>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/program_options.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cstddef>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <set>
#include <string>

#include "../config/detector.h"
#include "../log.h"
#include "../util/waveform_stream_id.h"
#include "../waveform.h"

namespace fs = boost::filesystem;
namespace po = boost::program_options;
namespace detect = Seiscomp::detect;

namespace Seiscomp {
namespace detect {
namespace perf {
namespace detail {

std::set<std::string> emergeWaveformStreamIds(std::ifstream &ifs) {
  std::set<std::string> ret;

  boost::property_tree::ptree pt;
  boost::property_tree::read_json(ifs, pt);

  config::DetectorConfig defaultDetectorConfig;
  config::StreamConfig defaultStreamConfig;
  config::PublishConfig defaulPublishConfig;
  for (const auto &templateSettingPt : pt) {
    config::TemplateConfig tc{templateSettingPt.second, defaultDetectorConfig,
                              defaultStreamConfig, defaulPublishConfig};

    for (const auto &streamConfigPair : tc) {
      ret.emplace(streamConfigPair.second.wfStreamId);
    }
  }
  return ret;
}

bool extractAndDumpWaveform(
    WaveformHandler &waveformHandler,
    const util::WaveformStreamID &waveformStreamId,
    const Seiscomp::Core::Time &startTime, const Seiscomp::Core::Time &endTime,
    const WaveformHandler::ProcessingConfig &processingConfig,
    std::ofstream &ofs) {
  try {
    Seiscomp::Core::TimeWindow tw;
    if (startTime) {
      tw.setStartTime(startTime);
    }

    if (endTime) {
      tw.setEndTime(endTime);
    } else {
      tw.setEndTime(Seiscomp::Core::Time::UTC());
    }
    auto record{waveformHandler.get(
        waveformStreamId.netCode(), waveformStreamId.staCode(),
        waveformStreamId.locCode(), waveformStreamId.chaCode(), tw,
        processingConfig)};

    // copy waveform data
    Seiscomp::GenericRecord copied{*record};
    // the caller is required to copy the data
    // https://github.com/SeisComP/common/issues/38
    copied.setData(dynamic_cast<Seiscomp::DoubleArray *>(
        record->data()->copy(Seiscomp::Array::DOUBLE)));

    if (!detect::waveform::write(copied, ofs)) {
      SCDETECT_LOG_ERROR("Failed to write data for stream: %s",
                         detect::util::to_string(waveformStreamId).c_str());
      return false;
    }

  } catch (detect::WaveformHandlerIface::BaseException &e) {
    SCDETECT_LOG_ERROR("Failed to load waveform data for stream: %s",
                       detect::util::to_string(waveformStreamId).c_str());
    return false;
  } catch (...) {
    SCDETECT_LOG_ERROR("Failed to extract waveform data for stream: %s",
                       detect::util::to_string(waveformStreamId).c_str());
    return false;
  }
  return true;
}

}  // namespace detail
}  // namespace perf
}  // namespace detect
}  // namespace Seiscomp

int main(int argc, char **argv) {
  // setup commandline arguments
  std::string startTimeStr;
  std::string endTimeStr;
  double targetFrequency{0};

  po::options_description generic{"Allowed options"};
  // clang-format off
  generic.add_options()
    ("help,h", "show this help message and exit")
    ("starttime", po::value<std::string>(&startTimeStr),
     "trim data to starttime")
    ("endtime", po::value<std::string>(&endTimeStr), "trim data to endtime")
    ("target-frequency", po::value<double>(&targetFrequency)->default_value(0),
     "resampling target frequency; if 0 no resampling is performed");
  // clang-format on

  std::string recordStreamURI;
  std::vector<fs::path> templateConfigPaths;
  po::options_description hidden{"Hidden options"};
  hidden.add_options()("record-uri", po::value<std::string>(&recordStreamURI),
                       "archive recordStream URI")(
      "templates-json", po::value<std::vector<fs::path>>(&templateConfigPaths),
      "path to template JSON configuraion");

  po::options_description all;
  all.add(generic).add(hidden);

  po::positional_options_description positionalOptions;
  positionalOptions.add("record-uri", 1);
  positionalOptions.add("templates-json", -1);

  // parse commandline
  po::variables_map vm;
  try {
    auto parsed{po::command_line_parser(argc, argv)
                    .options(all)
                    .positional(positionalOptions)
                    .run()};
    po::store(parsed, vm);
    po::notify(vm);
  } catch (const po::error &e) {
    std::cout << "ERROR: " << e.what() << "\n\n";
    std::cout << generic << std::endl;
    return EXIT_FAILURE;
  }

  if (vm.count("help")) {
    std::cout
        << "Utility to prepare scdetect-cc app benchmark waveform data\n\n";
    std::cout << generic << std::endl;
    return EXIT_SUCCESS;
  }

  // TODO(damb): configure logging. Currently, the log messages are not
  // displayed.

  if (templateConfigPaths.empty()) {
    SCDETECT_LOG_DEBUG("Nothing to do.");
    return EXIT_SUCCESS;
  }

  auto validateAndStoreTime = [](const std::string &timeStr,
                                 Seiscomp::Core::Time &result) {
    if (!timeStr.empty() && !result.fromString(timeStr.c_str(), "%FT%T")) {
      SCDETECT_LOG_ERROR("Invalid time: %s", timeStr.c_str());
      return false;
    }
    return true;
  };

  Seiscomp::Core::Time startTime;
  if (!startTimeStr.empty() && !validateAndStoreTime(startTimeStr, startTime)) {
    SCDETECT_LOG_ERROR("Failed to parse starttime: %s", startTimeStr.c_str());
    return EXIT_FAILURE;
  }

  Seiscomp::Core::Time endTime;
  if (!endTimeStr.empty() && !validateAndStoreTime(endTimeStr, endTime)) {
    SCDETECT_LOG_ERROR("Failed to parse endtime: %s", endTimeStr.c_str());
    return EXIT_FAILURE;
  }

  detect::WaveformHandler waveformHandler(recordStreamURI);
  detect::WaveformHandlerIface::ProcessingConfig processingConfig;
  processingConfig.demean = false;
  processingConfig.targetFrequency = targetFrequency;
  for (const auto &templateConfigPath : templateConfigPaths) {
    const auto p{fs::absolute(templateConfigPath)};
    SCDETECT_LOG_DEBUG("Reading template configuration from: %s ...",
                       p.c_str());
    std::set<std::string> waveformStreamIds;
    try {
      std::ifstream ifs{p.string()};
      waveformStreamIds = detect::perf::detail::emergeWaveformStreamIds(ifs);
    } catch (...) {
      SCDETECT_LOG_ERROR(
          "Failed to parse JSON template configuration file (%s).", p.c_str());
      continue;
    }

    for (const auto &_waveformStreamId : waveformStreamIds) {
      detect::util::WaveformStreamID waveformStreamId{_waveformStreamId};

      const auto fname{detect::util::to_string(waveformStreamId) + ".mseed"};
      const auto outPath{p.parent_path() /= fname};

      std::ofstream ofs{outPath.string()};
      if (!detect::perf::detail::extractAndDumpWaveform(
              waveformHandler, waveformStreamId, startTime, endTime,
              processingConfig, ofs)) {
        fs::remove(outPath);
      }
    }
  }
  return EXIT_SUCCESS;
}
