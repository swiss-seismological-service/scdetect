#define SEISCOMP_TEST_MODULE test_integration_general
#include <seiscomp/datamodel/eventparameters.h>
#include <seiscomp/io/archive/xmlarchive.h>
#include <seiscomp/unittest/unittests.h>

#include <boost/algorithm/string/join.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/test/data/dataset.hpp>
#include <boost/test/data/monomorphic.hpp>
#include <boost/test/data/test_case.hpp>
#include <boost/test/tools/fpc_tolerance.hpp>
#include <cstdlib>
#include <memory>
#include <ostream>
#include <string>
#include <vector>

#include "../app.h"
#include "fixture.h"
#include "integration_utils.h"

namespace utf = boost::unit_test;
namespace utf_data = utf::data;
namespace fs = boost::filesystem;

constexpr double testUnitTolerance{0.000001};

namespace Seiscomp {
namespace detect {
namespace test {

namespace ds {

struct Sample {
  std::string pathTemplateConfig;
  std::string pathInventory;
  std::string pathCatalog;
  std::string pathRecords;

  std::string startTime;

  std::string pathExpected;

  fs::path pathSample;

  using Flags = std::vector<std::shared_ptr<cli::Flag>>;
  Flags customFlags;

  std::vector<std::string> AsFlags(const fs::path &path_data) const {
    std::vector<std::string> flags{
        cli::to_string(cli::FlagTemplatesJSON{path_data / pathSample /
                                              pathTemplateConfig}),
        cli::to_string(
            cli::FlagInventoryDB{path_data / pathSample / pathInventory}),
        cli::to_string(cli::FlagRecordStartTime{startTime}),
        cli::to_string(cli::FlagRecordURL{
            "file://" + (path_data / pathSample / pathRecords).string()}),
        cli::to_string(cli::FlagEventDB{path_data / pathSample / pathCatalog}),
    };

    // serialize custom flags
    for (const auto &flag : customFlags) {
      flags.push_back(cli::to_string(*flag));
    }

    return flags;
  }

  friend std::ostream &operator<<(std::ostream &os, const Sample &sample);
};

std::ostream &operator<<(std::ostream &os, const Sample &sample) {
  auto samplePath = [&sample](const std::string &fname) {
    return fname.empty() ? "" : (sample.pathSample / fname).string();
  };

  return os << "templateConfig: " << samplePath(sample.pathTemplateConfig)
            << ", inventory: " << samplePath(sample.pathInventory)
            << ", catalog: " << samplePath(sample.pathCatalog)
            << ", starttime: " << sample.startTime
            << ", records: " << samplePath(sample.pathRecords)
            << ", expected: " << samplePath(sample.pathExpected);
}

}  // namespace ds

// samples for parameterized testing
using Samples = std::vector<ds::Sample>;
Samples dataset{
    // base: single detector - single stream
    {"templates.json",
     "inventory.scml",
     "catalog.scml",
     "data.mseed",
     /*starttime=*/"2020-10-25T19:30:00",
     "expected.scml",
     /*pathSample=*/"integration/base/single-detector-single-stream-0000",
     /*customFlags=*/
     {std::make_shared<cli::FlagAmplitudesForce>(false),
      std::make_shared<cli::FlagMagnitudesForce>(false)}},
    {"templates.json",
     "inventory.scml",
     "catalog.scml",
     "data.mseed",
     /*starttime=*/"2020-10-25T19:30:00",
     "expected.scml",
     /*pathSample=*/"integration/base/single-detector-single-stream-0001",
     /*customFlags=*/
     {std::make_shared<cli::FlagAmplitudesForce>(false),
      std::make_shared<cli::FlagMagnitudesForce>(false)}},
    {"templates.json",
     "inventory.scml",
     "catalog.scml",
     "data.mseed",
     /*starttime=*/"2020-10-25T19:30:00",
     "expected.scml",
     /*pathSample=*/"integration/base/single-detector-single-stream-0002",
     /*customFlags=*/
     {std::make_shared<cli::FlagAmplitudesForce>(false),
      std::make_shared<cli::FlagMagnitudesForce>(false)}},
    {"templates.json",
     "inventory.scml",
     "catalog.scml",
     "data.mseed",
     /*starttime=*/"2020-10-25T19:30:00",
     "expected.scml",
     /*pathSample=*/"integration/base/single-detector-single-stream-0003",
     /*customFlags=*/
     {std::make_shared<cli::FlagAmplitudesForce>(false),
      std::make_shared<cli::FlagMagnitudesForce>(false)}},
    {"templates.json",
     "inventory.scml",
     "catalog.scml",
     "data.mseed",
     /*starttime=*/"2020-10-25T19:30:00",
     "expected.scml",
     /*pathSample=*/"integration/base/single-detector-single-stream-0004",
     /*customFlags=*/
     {std::make_shared<cli::FlagAmplitudesForce>(false),
      std::make_shared<cli::FlagMagnitudesForce>(false)}},
    {"templates.json",
     "inventory.scml",
     "catalog.scml",
     "data.mseed",
     /*starttime=*/"2020-10-25T19:30:00",
     "expected.scml",
     /*pathSample=*/"integration/base/single-detector-single-stream-0005",
     /*customFlags=*/
     {std::make_shared<cli::FlagAmplitudesForce>(false),
      std::make_shared<cli::FlagMagnitudesForce>(false)}},
    {"templates.json",
     "inventory.scml",
     "catalog.scml",
     "data.mseed",
     /*starttime=*/"2020-10-25T19:30:00",
     "expected.scml",
     /*pathSample=*/"integration/base/single-detector-single-stream-0006",
     /*customFlags=*/
     {std::make_shared<cli::FlagAmplitudesForce>(false),
      std::make_shared<cli::FlagMagnitudesForce>(false)}},
    {"templates.json",
     "inventory.scml",
     "catalog.scml",
     "data.mseed",
     /*starttime=*/"2020-10-25T19:30:00",
     "expected.scml",
     /*pathSample=*/"integration/base/single-detector-single-stream-0007",
     /*customFlags=*/
     {std::make_shared<cli::FlagAmplitudesForce>(false),
      std::make_shared<cli::FlagMagnitudesForce>(false)}},

    // base: single detector - multi stream
    {"templates.json",
     "inventory.scml",
     "catalog.scml",
     "data.mseed",
     /*starttime=*/"2020-10-25T19:30:00",
     "expected.scml",
     /*pathSample=*/"integration/base/single-detector-multi-stream-0000",
     /*customFlags=*/
     {std::make_shared<cli::FlagAmplitudesForce>(false),
      std::make_shared<cli::FlagMagnitudesForce>(false)}},
    {"templates.json",
     "inventory.scml",
     "catalog.scml",
     "data.mseed",
     /*starttime=*/"2019-11-05T05:10:00",
     "expected.scml",
     /*pathSample=*/"integration/base/single-detector-multi-stream-0001",
     /*customFlags=*/
     {std::make_shared<cli::FlagAmplitudesForce>(false),
      std::make_shared<cli::FlagMagnitudesForce>(false)}},

    // base: multi detector - single stream
    {"templates.json",
     "inventory.scml",
     "catalog.scml",
     "data.mseed",
     /*starttime=*/"2020-10-25T19:30:00",
     "expected.scml",
     /*pathSample=*/"integration/base/multi-detector-single-stream-0000",
     /*customFlags=*/
     {std::make_shared<cli::FlagAmplitudesForce>(false),
      std::make_shared<cli::FlagMagnitudesForce>(false)}},

    // detector: single detector - multi stream
    {"templates.json",
     "inventory.scml",
     "catalog.scml",
     "data.mseed",
     /*starttime=*/"2020-10-25T20:30:00",
     "expected.scml",
     /*pathSample=*/"integration/detector/single-detector-multi-stream-0000",
     /*customFlags=*/
     {std::make_shared<cli::FlagAmplitudesForce>(false),
      std::make_shared<cli::FlagMagnitudesForce>(false)}},
    {"templates.json",
     "inventory.scml",
     "catalog.scml",
     "data.mseed",
     /*starttime=*/"2020-10-25T20:30:00",
     "expected.scml",
     /*pathSample=*/
     "integration/detector/single-detector-multi-stream-0001",
     /*customFlags=*/
     {std::make_shared<cli::FlagAmplitudesForce>(false),
      std::make_shared<cli::FlagMagnitudesForce>(false)}},
    {"templates.json",
     "inventory.scml",
     "catalog.scml",
     "data.mseed",
     /*starttime=*/"2020-10-25T20:30:00",
     "expected.scml",
     /*pathSample=*/"integration/detector/single-detector-multi-stream-0002",
     /*customFlags=*/
     {std::make_shared<cli::FlagAmplitudesForce>(false),
      std::make_shared<cli::FlagMagnitudesForce>(false)}},
    {"templates.json",
     "inventory.scml",
     "catalog.scml",
     "data.mseed",
     /*starttime=*/"2020-10-25T19:30:00",
     "expected.scml",
     /*pathSample=*/
     "integration/detector/single-detector-multi-stream-0003",
     /*customFlags=*/
     {std::make_shared<cli::FlagAmplitudesForce>(false),
      std::make_shared<cli::FlagMagnitudesForce>(false)}},
    {"templates.json",
     "inventory.scml",
     "catalog.scml",
     "data.mseed",
     /*starttime=*/"2020-10-25T19:30:00",
     "expected.scml",
     /*pathSample=*/"integration/detector/single-detector-multi-stream-0004",
     /*customFlags=*/
     {std::make_shared<cli::FlagAmplitudesForce>(false),
      std::make_shared<cli::FlagMagnitudesForce>(false)}},
    {"templates.json",
     "inventory.scml",
     "catalog.scml",
     "data.mseed",
     /*starttime=*/"2019-11-05T05:10:00",
     "expected.scml",
     /*pathSample=*/"integration/detector/single-detector-multi-stream-0005",
     /*customFlags=*/
     {std::make_shared<cli::FlagAmplitudesForce>(false),
      std::make_shared<cli::FlagMagnitudesForce>(false)}},

    // processing: resample
    {"templates.json",
     "inventory.scml",
     "catalog.scml",
     "data.mseed",
     /*starttime=*/"2020-10-25T19:30:00",
     "expected.scml",
     /*pathSample=*/
     "integration/processing/resample/single-detector-single-stream-0000",
     /*customFlags=*/
     {std::make_shared<cli::FlagAmplitudesForce>(false),
      std::make_shared<cli::FlagMagnitudesForce>(false)}},
    {"templates.json",
     "inventory.scml",
     "catalog.scml",
     "data.mseed",
     /*starttime=*/"2020-10-25T20:20:00",
     "expected.scml",
     /*pathSample=*/
     "integration/processing/resample/single-detector-single-stream-0001",
     /*customFlags=*/
     {std::make_shared<cli::FlagAmplitudesForce>(false),
      std::make_shared<cli::FlagMagnitudesForce>(false)}},

    // processing: changing sampling frequency
    {"templates.json",
     "inventory.scml",
     "catalog.scml",
     "data.mseed",
     /*starttime=*/"2020-10-25T20:20:00",
     "expected.scml",
     /*pathSample=*/
     "integration/processing/changing-fsamp/"
     "single-detector-single-stream-0000",
     /*customFlags=*/
     {std::make_shared<cli::FlagAmplitudesForce>(false),
      std::make_shared<cli::FlagMagnitudesForce>(false)}},
    {"templates.json",
     "inventory.scml",
     "catalog.scml",
     "data.mseed",
     /*starttime=*/"2020-10-25T20:20:00",
     "expected.scml",
     /*pathSample=*/
     "integration/processing/changing-fsamp/"
     "single-detector-single-stream-0001",
     /*customFlags=*/
     {std::make_shared<cli::FlagAmplitudesForce>(false),
      std::make_shared<cli::FlagMagnitudesForce>(false)}},
    {"templates.json",
     "inventory.scml",
     "catalog.scml",
     "data.mseed",
     /*starttime=*/"2020-10-25T20:20:00",
     "expected.scml",
     /*pathSample=*/
     "integration/processing/changing-fsamp/"
     "single-detector-single-stream-0002",
     /*customFlags=*/
     {std::make_shared<cli::FlagAmplitudesForce>(false),
      std::make_shared<cli::FlagMagnitudesForce>(false)}},
    {"templates.json",
     "inventory.scml",
     "catalog.scml",
     "data.mseed",
     /*starttime=*/"2020-10-25T20:20:00",
     "expected.scml",
     /*pathSample=*/
     "integration/processing/changing-fsamp/"
     "single-detector-single-stream-0003",
     /*customFlags=*/
     {std::make_shared<cli::FlagAmplitudesForce>(false),
      std::make_shared<cli::FlagMagnitudesForce>(false)}},
    {"templates.json",
     "inventory.scml",
     "catalog.scml",
     "data.mseed",
     /*starttime=*/"2020-10-25T20:20:00",
     "expected.scml",
     /*pathSample=*/
     "integration/processing/changing-fsamp/"
     "single-detector-single-stream-0004",
     /*customFlags=*/
     {std::make_shared<cli::FlagAmplitudesForce>(false),
      std::make_shared<cli::FlagMagnitudesForce>(false)}},
};

BOOST_TEST_GLOBAL_FIXTURE(CLIParserFixture);

BOOST_TEST_DECORATOR(*utf::tolerance(testUnitTolerance))
BOOST_DATA_TEST_CASE(integration, utf_data::make(dataset)) {
  TempDirFixture fx{CLIParserFixture::keepTempdir};
  // prepare empty config file
  fs::path pathConfig{fx.pathTempdir / "scdetect.cfg"};
  try {
    fs::ofstream{pathConfig};
  } catch (fs::filesystem_error &e) {
    BOOST_FAIL("Failed to prepare dummy config file: " << e.what());
  }

  fs::path pathEpResultSCML{fx.pathTempdir / "ep.scml"};

  // prepare CLI flags
  std::vector<std::string> flagsStr{
      "scdetect",
      cli::to_string(cli::FlagConfigFile{pathConfig}),
      cli::to_string(cli::FlagDebug{}),
      cli::to_string(cli::FlagOffline{}),
      cli::to_string(cli::FlagPlayback{}),
      cli::to_string(cli::FlagTemplatesReload{}),
      cli::to_string(cli::FlagEp{pathEpResultSCML}),
      cli::to_string(cli::FlagAgencyId{"TEST"})};
  auto flagsSample{sample.AsFlags(CLIParserFixture::pathData)};
  flagsStr.insert(std::end(flagsStr), std::begin(flagsSample),
                  std::end(flagsSample));

  BOOST_TEST_MESSAGE("Running integration test with CLI args: "
                     << boost::algorithm::join(flagsStr, " "));
  BOOST_TEST_MESSAGE("Path to temporary test data: " << fx.pathTempdir);
  try {
    // parse README
    std::string readmeHeader;
    fs::ifstream ifs{CLIParserFixture::pathData / sample.pathSample / "README"};
    getline(ifs, readmeHeader);
    if (!readmeHeader.empty()) {
      BOOST_TEST_MESSAGE("Test purpose: " << readmeHeader);
      getline(ifs, readmeHeader);
      std::stringstream buffer;
      if (buffer << ifs.rdbuf()) {
        BOOST_TEST_MESSAGE("Test description and configuration:\n\n"
                           << buffer.str());
      }
    }
  } catch (fs::filesystem_error &e) {
  }

  int retval{EXIT_FAILURE};
  { retval = ApplicationWrapper<Application>{flagsStr}(); }

  BOOST_TEST_CHECK(EXIT_SUCCESS == retval);

  auto readEventParameters = [](const fs::path &path,
                                DataModel::EventParametersPtr &ep_ptr) {
    BOOST_TEST_REQUIRE(!path.empty(), "Invalid path.");
    IO::XMLArchive ar;
    if (!ar.open(path.c_str())) {
      BOOST_FAIL("Failed to open file: " << path);
    }
    ar >> ep_ptr;
    ar.close();
  };

  // read detection results
  DataModel::EventParametersPtr epResult;
  readEventParameters(pathEpResultSCML, epResult);
  BOOST_TEST_REQUIRE(epResult, "Failed to read file: " << pathEpResultSCML);

  // read expected result
  DataModel::EventParametersPtr epExpected;
  fs::path pathEpExpectedSCML{CLIParserFixture::pathData / sample.pathSample /
                              sample.pathExpected};
  readEventParameters(pathEpExpectedSCML, epExpected);
  BOOST_TEST_REQUIRE(epExpected, "Failed to read file: " << pathEpExpectedSCML);

  eventParametersCmp(epResult, epExpected);
}

}  // namespace test
}  // namespace detect
}  // namespace Seiscomp
