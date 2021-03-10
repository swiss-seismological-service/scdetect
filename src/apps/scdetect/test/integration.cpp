#define SEISCOMP_TEST_MODULE test_integration_general
#include <seiscomp/unittest/unittests.h>

#include <cstdlib>
#include <ostream>
#include <string>
#include <vector>

#include <boost/algorithm/string/join.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/test/data/dataset.hpp>
#include <boost/test/data/monomorphic.hpp>
#include <boost/test/data/test_case.hpp>
#include <boost/test/tools/fpc_tolerance.hpp>

#include <seiscomp/datamodel/eventparameters.h>
#include <seiscomp/io/archive/xmlarchive.h>

#include "integration_utils.h"

#include "../app.h"

namespace utf = boost::unit_test;
namespace utf_data = utf::data;
namespace utf_tt = boost::test_tools;
namespace fs = boost::filesystem;

constexpr double test_unit_tolerance{0.000001};

namespace Seiscomp {
namespace detect {
namespace test {

namespace ds {

struct Sample {
  std::string path_template_config;
  std::string path_inventory;
  std::string path_catalog;
  std::string path_records;

  std::string starttime;

  std::string path_expected;

  fs::path path_sample;

  std::vector<std::string> AsFlags(const fs::path &path_data) const {
    std::vector<std::string> flags{
        cli::to_string(cli::FlagTemplatesJSON{path_data / path_sample /
                                              path_template_config}),
        cli::to_string(
            cli::FlagInventoryDB{path_data / path_sample / path_inventory}),
        cli::to_string(cli::FlagRecordStartTime{starttime}),
        cli::to_string(cli::FlagRecordURL{
            "file://" + (path_data / path_sample / path_records).string()}),
        cli::to_string(
            cli::FlagEventDB{path_data / path_sample / path_catalog}),
    };

    return flags;
  }

  friend std::ostream &operator<<(std::ostream &os, const Sample &sample);
};

std::ostream &operator<<(std::ostream &os, const Sample &sample) {
  auto SamplePath = [&sample](const std::string &fname) {
    return fname.empty() ? "" : (sample.path_sample / fname).string();
  };

  return os << "template_config: " << SamplePath(sample.path_template_config)
            << ", inventory: " << SamplePath(sample.path_inventory)
            << ", catalog: " << SamplePath(sample.path_catalog)
            << ", starttime: " << sample.starttime
            << ", records: " << SamplePath(sample.path_records)
            << ", expected: " << SamplePath(sample.path_expected);
}

} // namespace ds

// samples for parameterized testing
using Samples = std::vector<ds::Sample>;
Samples dataset{
    {"templates.json", "inventory.scml", "catalog.scml", "data.mseed",
     /*starttime=*/"2020-10-25T19:30:00", "expected.scml",
     /*path_sample=*/"integration-single-stream-simple-without-arrivals"},
    {"templates.json", "inventory.scml", "catalog.scml", "data.mseed",
     /*starttime=*/"2020-10-25T19:30:00", "expected.scml",
     /*path_sample=*/"integration-single-stream-simple-with-arrivals-00"},
    {"templates.json", "inventory.scml", "catalog.scml", "data.mseed",
     /*starttime=*/"2020-10-25T19:30:00", "expected.scml",
     /*path_sample=*/"integration-single-stream-simple-with-templ-arrivals"},
    {"templates.json", "inventory.scml", "catalog.scml", "data.mseed",
     /*starttime=*/"2020-10-25T19:30:00", "expected.scml",
     /*path_sample=*/"integration-single-stream-filtered"},
    {"templates.json", "inventory.scml", "catalog.scml", "data.mseed",
     /*starttime=*/"2020-10-25T19:30:00", "expected.scml",
     /*path_sample=*/
     "integration-multi-detector-single-stream-simple"},
    {"templates.json", "inventory.scml", "catalog.scml", "data.mseed",
     /*starttime=*/"2020-10-25T19:30:00", "expected.scml",
     /*path_sample=*/
     "integration-multi-stream-simple"},
    {"templates.json", "inventory.scml", "catalog.scml", "data.mseed",
     /*starttime=*/"2020-10-25T19:30:00", "expected.scml",
     /*path_sample=*/
     "integration-multi-stream-templ-wf-diff-00"},
    {"templates.json", "inventory.scml", "catalog.scml", "data.mseed",
     /*starttime=*/"2020-10-25T19:30:00", "expected.scml",
     /*path_sample=*/
     "integration-multi-stream-templ-wf-diff-01"},
    {"templates.json", "inventory.scml", "catalog.scml", "data.mseed",
     /*starttime=*/"2020-10-25T20:30:00", "expected.scml",
     /*path_sample=*/
     "integration-multi-stream-arrival-offset-00"},
    {"templates.json", "inventory.scml", "catalog.scml", "data.mseed",
     /*starttime=*/"2020-10-25T20:30:00", "expected.scml",
     /*path_sample=*/
     "integration-multi-stream-arrival-offset-01"},
    {"templates.json", "inventory.scml", "catalog.scml", "data.mseed",
     /*starttime=*/"2020-10-25T20:30:00", "expected.scml",
     /*path_sample=*/
     "integration-multi-stream-min-arrivals"},
};

BOOST_TEST_GLOBAL_FIXTURE(CLIParserFixture);

BOOST_TEST_DECORATOR(*utf::tolerance(test_unit_tolerance))
BOOST_DATA_TEST_CASE(integration, utf_data::make(dataset)) {

  TempDirFixture fx{CLIParserFixture::keep_tempdir};

  // prepare empty SQLite DB from template DB
  const std::string db{"seiscomp_db.sqlite"};
  fs::path path_db{fx.path_tempdir / db};
  try {
    fs::copy_file(CLIParserFixture::path_data / db, path_db);
  } catch (fs::filesystem_error &e) {
    BOOST_FAIL("Failed to prepare database:" << e.what());
  }

  fs::path path_ep_result_scml{fx.path_tempdir / "ep.scml"};

  // prepare CLI flags
  std::vector<std::string> flags_str{
      "scdetect",
      cli::to_string(cli::FlagDebug{}),
      cli::to_string(cli::FlagOffline{}),
      cli::to_string(cli::FlagPlayback{}),
      cli::to_string(cli::FlagTemplatesReload{}),
      cli::to_string(cli::FlagPlugins{"dbsqlite3"}),
      cli::to_string(cli::FlagDB{path_db}),
      cli::to_string(cli::FlagEp{path_ep_result_scml}),
      cli::to_string(cli::FlagAgencyId{"TEST"})};
  auto flags_sample{sample.AsFlags(CLIParserFixture::path_data)};
  flags_str.insert(std::end(flags_str), std::begin(flags_sample),
                   std::end(flags_sample));

  BOOST_TEST_MESSAGE("Running integration test with CLI args: "
                     << boost::algorithm::join(flags_str, " "));
  BOOST_TEST_MESSAGE("Path to temporary test data: " << fx.path_tempdir);

  int retval{EXIT_FAILURE};
  { retval = ApplicationWrapper<Application>{flags_str}(); }

  BOOST_TEST_CHECK(EXIT_SUCCESS == retval);

  auto ReadEventParameters = [](const fs::path &path,
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
  DataModel::EventParametersPtr ep_result;
  ReadEventParameters(path_ep_result_scml, ep_result);
  BOOST_TEST_REQUIRE(ep_result, "Failed to read file: " << path_ep_result_scml);

  // read expected result
  DataModel::EventParametersPtr ep_expected;
  fs::path path_ep_expected_scml{CLIParserFixture::path_data /
                                 sample.path_sample / sample.path_expected};
  ReadEventParameters(path_ep_expected_scml, ep_expected);
  BOOST_TEST_REQUIRE(ep_expected,
                     "Failed to read file: " << path_ep_expected_scml);

  EventParametersCmp(ep_result, ep_expected);
}

} // namespace test
} // namespace detect
} // namespace Seiscomp
