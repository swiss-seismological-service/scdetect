#include "../app.h"

#include <algorithm>
#include <boost/algorithm/string/join.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/program_options/errors.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/positional_options.hpp>
#include <boost/program_options/value_semantic.hpp>
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <ostream>
#include <string>
#include <vector>

#include "perf.h"

namespace fs = boost::filesystem;
namespace po = boost::program_options;

namespace Seiscomp {
namespace detect {
namespace perf {

const std::string kScdetectCCBinary{"scdetect-cc"};

class PerfApplication : public Application {
 public:
  PerfApplication(int argc, char **argv, std::size_t trials)
      : Application{argc, argv}, _trials{trials} {
    setAutoAcquisitionStart(false);
  }
  ~PerfApplication() override = default;

  const PerfTimer &perfTimer() const { return _timer; }

  using Application::detectors;

 protected:
  bool run() override {
    for (std::size_t trial{0}; trial < _trials; ++trial) {
      if (static_cast<bool>(recordStream())) {
        recordStream()->close();
      }
      waitForRecordThread();
      closeStream();
      reloadRecordStream();
      resetDetectors();
      startRecordThread();
      _exitRequested = false;
      _currentTrial = trial + 1;

      _timer.start();
      if (!detect::Application::run()) {
        return false;
      }
      _timer.stop();
    }

    return true;
  }

  void handleEndAcquisition() override {
    if (_currentTrial >= _trials) {
      sendNotification(Client::Notification::Close);
    } else {
      _exitRequested = true;
    }
  }

 private:
  bool reloadRecordStream() {
    const auto tmp{Client::Application::_settings.recordstream.URI};
    try {
      std::string inputFile{commandline().option<std::string>("record-file")};
      std::string type;

      try {
        type = commandline().option<std::string>("record-type");
      } catch (...) {
      }

      if (!inputFile.empty()) {
        fs::path p{inputFile};
        if (!fs::is_regular_file(p)) {
          return false;
        }

        // XXX(damb): this is a workaround. Since
        // StreamApplication::_recordStream is a private member resetting the
        // record stream url is required.
        Client::Application::_settings.recordstream.URI =
            "file://" + fs::absolute(p).string();
        openStream();
        if (!recordStream()) {
          SEISCOMP_ERROR("Failed to create recordstream from file: \"%s\"",
                         p.c_str());
          return false;
        }
        if (!type.empty()) {
          if (!recordStream()->setRecordType(type.c_str())) {
            SEISCOMP_ERROR("Failed to set recordtype to \"%s\"", type.c_str());
            return false;
          }
        }
      }
    } catch (...) {
      Client::Application::_settings.recordstream.URI = tmp;
      openStream();
    }

    if (!recordStream()) {
      SEISCOMP_ERROR("Failed to open recordstream: \"%s\"",
                     recordStreamURL().c_str());
      return false;
    }
    return true;
  }

  PerfTimer _timer;
  std::size_t _trials;
  std::size_t _currentTrial{};
};

double perfApplication(const std::vector<std::string> &argv,
                       std::size_t trials) {
  auto strToCStr = [](const std::string &str) {
    char *ret{new char[str.size() + 1]};
    std::strcpy(ret, str.c_str());
    return ret;
  };

  std::vector<std::string> copied{kScdetectCCBinary};
  copied.insert(std::end(copied), std::begin(argv), std::end(argv));

  std::vector<char *> transformed;
  transformed.reserve(argv.size());
  std::transform(std::begin(copied), std::end(copied),
                 back_inserter(transformed), strToCStr);

  PerfApplication app{static_cast<int>(transformed.size()), transformed.data(),
                      trials};

  app.exec();

  for (const auto &detector : app.detectors()) {
    for (const auto &templateWaveformProcessor : *detector) {
      std::cout << "template waveform samples ["
                << templateWaveformProcessor.id()
                << "]: " << templateWaveformProcessor.templateWaveform().size()
                << std::endl;
    }
  }

  for (std::size_t i{0}; i < transformed.size(); ++i) {
    delete[] transformed[i];
  }

  return app.perfTimer().trials() > 0 ? app.perfTimer().minTime() : 0;
}

}  // namespace perf
}  // namespace detect
}  // namespace Seiscomp

int main(int argc, char **argv) {
  // setup commandline arguments
  std::size_t trials;

  po::options_description generic{"Allowed options"};
  generic.add_options()("help,h", "show this help message and exit")(
      "trials", po::value<std::size_t>(&trials)->default_value(3),
      "number of trials to run");

  std::vector<std::string> cmd;
  po::options_description hidden{"Hidden options"};
  hidden.add_options()("cmd", po::value<std::vector<std::string>>(&cmd),
                       "command to forward");

  po::options_description all;
  all.add(generic).add(hidden);

  po::positional_options_description positionalOptions;
  positionalOptions.add("cmd", -1);

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
    std::cout << "ERROR: " << e.what() << std::endl;
    std::cout << generic << std::endl;
    return EXIT_FAILURE;
  }

  if (vm.count("help")) {
    std::cout << generic << std::endl;
    return EXIT_SUCCESS;
  }

  std::cout << "trials: " << trials << std::endl;
  std::cout << "cmd: '" << boost::join(cmd, " ") << "'" << std::endl;

  auto itRecordStream{
      std::find_if(std::begin(cmd), std::end(cmd), [](const std::string &s) {
        return (s.rfind("-I=", 0) != std::string::npos) ||
               (s.rfind("--record-url=", 0) != std::string::npos);
      })};

  if (itRecordStream == std::end(cmd)) {
    std::cout << "ERROR: missing recordStream flag" << std::endl;
    return EXIT_FAILURE;
  }

  std::string recordStreamURL{
      ++std::find(std::begin(*itRecordStream), std::end(*itRecordStream), '='),
      std::end(*itRecordStream)};

  std::cout << "record-url: " << recordStreamURL << std::endl;
  std::unique_ptr<Seiscomp::IO::RecordStream> rs{
      Seiscomp::IO::RecordStream::Open(recordStreamURL.c_str())};

  std::size_t numRecords{};
  std::size_t numSamples{};
  double samplingFrequency{0};
  if (rs) {
    /* rs->addStream("*", "*", "*", "*"); */
    while (std::unique_ptr<Seiscomp::Record> rec{rs->next()}) {
      numSamples += rec->sampleCount();
      ++numRecords;
      if (0 == samplingFrequency) {
        samplingFrequency = rec->samplingFrequency();
      } else if (samplingFrequency != rec->samplingFrequency()) {
        // force records to be passed with a unique sampling frequency
        std::cout << "ERROR: non-unique sampling frequency" << std::endl;
        return EXIT_FAILURE;
      }
    }
    rs->close();
  }

  std::cout << "records (total): " << numRecords << std::endl;
  std::cout << "samples (total): " << numSamples << std::endl;
  std::cout << "sampling frequency: " << samplingFrequency << " Hz"
            << std::endl;

  auto t{Seiscomp::detect::perf::perfApplication(cmd, trials)};
  std::cout << "time: " << t / 1e6 << " ms" << std::endl;

  return EXIT_SUCCESS;
}
