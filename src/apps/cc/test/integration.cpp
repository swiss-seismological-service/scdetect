#define SEISCOMP_TEST_MODULE test_cc_integration
#include <seiscomp/datamodel/eventparameters.h>
#include <seiscomp/io/archive/xmlarchive.h>
#include <seiscomp/unittest/unittests.h>

#include <boost/algorithm/string/join.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/optional/optional.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/positional_options.hpp>
#include <boost/program_options/value_semantic.hpp>
#include <boost/test/data/dataset.hpp>
#include <boost/test/data/monomorphic.hpp>
#include <boost/test/data/test_case.hpp>
#include <boost/test/tools/fpc_tolerance.hpp>
#include <cstddef>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "../app.h"
#include "fixture.h"
#include "integration_utils.h"

namespace utf = boost::unit_test;
namespace fs = boost::filesystem;
namespace po = boost::program_options;

constexpr double testUnitTolerance{0.000001};

namespace Seiscomp {
namespace detect {
namespace test {

namespace ds {

// A file based dataset (using Boost.Test's delayed dataset initialization)
//
// - allows the configuration to be read either from a file or from stdin
class FileBasedDataSet {
 public:
  static const int arity = 1;

  using CustomFlags = boost::optional<std::string>;
  using PathExpected = fs::path;

  FileBasedDataSet(std::size_t lineBegin = 0,
                   std::size_t lineEnd = static_cast<std::size_t>(-1))
      : _lineBegin{lineBegin}, _lineEnd{lineEnd} {
    try {
      po::options_description desc;
      desc.add_options()("path-data", po::value<fs::path>(&_pathData),
                         "Base path to the test data")(

          "test-dataset", po::value<fs::path>(&_pathTestDataSet),
          "Path to file containing the test data set configuration");

      po::positional_options_description pdesc;
      pdesc.add("test-dataset", -1);

      po::variables_map vm;
      po::store(
          po::command_line_parser(utf::framework::master_test_suite().argc,

                                  utf::framework::master_test_suite().argv)

              .options(desc)
              .positional(pdesc)
              .run(),
          vm);
      po::notify(vm);
    } catch (std::exception &e) {
      throw std::logic_error{"failed to parse commandline arguments"};
    }

    _pathTestDataSet = fs::absolute(_pathTestDataSet);
    std::ifstream ifs{_pathTestDataSet.string()};
    if (!ifs.is_open()) {
      throw std::logic_error{"cannot open the file '" +
                             _pathTestDataSet.string() + "'"};
    }

    auto countLines = [](std::istream &in) -> std::size_t {
      return std::count_if(std::istreambuf_iterator<char>(in),
                           std::istreambuf_iterator<char>(),
                           [](char c) { return c == '\n'; });
    };
    _lineEnd = std::min(countLines(ifs), _lineEnd);

    if (!(_lineBegin <= _lineEnd)) {
      throw std::logic_error{"incorrect line start/end"};
    }
  }

  struct Sample {
    fs::path pathTemplateConfig;
    fs::path pathInventoryDB;
    fs::path pathEventDB;
    fs::path pathWaveformData;

    boost::optional<fs::path> pathConfigDB;

    std::string startTime;

    fs::path pathExpected;

    using CustomFlags = boost::optional<std::string>;
    CustomFlags customFlags;

    std::vector<std::string> asFlags() const {
      std::vector<std::string> ret;
      ret.emplace_back(
          cli::to_string(cli::FlagTemplatesJSON{pathTemplateConfig}));
      ret.emplace_back(cli::to_string(cli::FlagInventoryDB{pathInventoryDB}));
      ret.emplace_back(cli::to_string(cli::FlagEventDB{pathEventDB}));
      ret.emplace_back(cli::to_string(cli::FlagRecordURL{pathWaveformData}));
      ret.emplace_back(cli::to_string(cli::FlagRecordStartTime{startTime}));
      if (pathConfigDB) {
        ret.emplace_back(cli::to_string(cli::FlagConfigDB{*pathConfigDB}));
      }

      if (customFlags) {
        ret.emplace_back(*customFlags);
      }

      return ret;
    }

    friend std::ostream &operator<<(std::ostream &os, const Sample &s) {
      return os << boost::algorithm::join(s.asFlags(), " ");
    }
  };

  struct Iterator {
    Iterator(const std::string &filename, std::size_t lineBegin,
             const fs::path &absPathData)
        : _ifs{filename, std::ios::binary}, _absBasePath{absPathData} {
      if (!_ifs) {
        throw std::runtime_error{"cannot open the file '" + filename + "'"};
      }

      for (std::size_t i = 0; i <= lineBegin; ++i) {
        std::getline(_ifs, _currentLine);
      }
    }

    auto operator*() const -> Sample {
      std::vector<std::string> tokens;
      std::istringstream iss{_currentLine};
      std::string token;

      while (std::getline(iss, token, '|')) {
        tokens.push_back(token);
      }

      if (tokens.size() < 8) {
        throw std::logic_error{
            "invalid sample definition: invalid number of tokens (" +
            _currentLine + ")"};
      }

      auto samplePath = fs::absolute(_absBasePath / tokens[0]);
      // validate
      if (!fs::is_directory(samplePath)) {
        throw std::logic_error{
            "invalid sample definition: invalid sample directory path: " +
            samplePath.string()};
      }

      Sample sample;
      sample.pathTemplateConfig = samplePath / tokens[1];
      sample.pathInventoryDB = samplePath / tokens[2];
      sample.pathEventDB = samplePath / tokens[3];
      sample.pathWaveformData = samplePath / tokens[4];
      if (!tokens[5].empty()) {
        sample.pathConfigDB = samplePath / tokens[5];
      }
      sample.startTime = tokens[6];
      sample.pathExpected = samplePath / tokens[7];

      if (tokens.size() > 8) {
        boost::algorithm::trim(tokens[8]);
        if (!tokens[8].empty()) {
          sample.customFlags = tokens[8];
        }
      }

      return sample;
    }

    void operator++() { std::getline(_ifs, _currentLine); }

   private:
    std::ifstream _ifs;
    fs::path _absBasePath{""};
    std::string _currentLine;
  };

  // Returns the size of the dataset
  boost::unit_test::data::size_t size() const { return _lineEnd - _lineBegin; }

  // Returns an iterator over the lines of the file
  Iterator begin() const {
    return Iterator{_pathTestDataSet.string(), _lineBegin, _pathData};
  }

 private:
  fs::path _pathTestDataSet;
  // Path to the base data directory
  fs::path _pathData;
  std::size_t _lineBegin;
  std::size_t _lineEnd;
};

}  // namespace ds
}  // namespace test
}  // namespace detect
}  // namespace Seiscomp

namespace boost {
namespace unit_test {
namespace data {
namespace monomorphic {

template <>
struct is_dataset<Seiscomp::detect::test::ds::FileBasedDataSet>
    : boost::mpl::true_ {};

}  // namespace monomorphic
}  // namespace data
}  // namespace unit_test
}  // namespace boost

namespace Seiscomp {
namespace detect {
namespace test {

BOOST_TEST_GLOBAL_FIXTURE(CLIParserFixture);

BOOST_TEST_DECORATOR(*utf::tolerance(testUnitTolerance))
BOOST_DATA_TEST_CASE(
    cc_integration,
    boost::unit_test::data::make_delayed<ds::FileBasedDataSet>()) {
  TempDirFixture fx{CLIParserFixture::keepTempdir};
  // prepare empty config file
  fs::path pathConfig{fx.pathTempdir / "scdetect-cc.cfg"};
  std::ofstream ofs{pathConfig.string()};
  if (!ofs) {
    BOOST_FAIL("Failed to prepare dummy config file: " << pathConfig);
  }

  fs::path pathEpResultSCML{fx.pathTempdir / "ep.scml"};

  // prepare CLI flags
  std::vector<std::string> flagsStr{
      "scdetect-cc",
      cli::to_string(cli::FlagConfigFile{pathConfig}),
      cli::to_string(cli::FlagDebug{}),
      cli::to_string(cli::FlagOffline{}),
      cli::to_string(cli::FlagPlayback{}),
      cli::to_string(cli::FlagTemplatesReload{}),
      cli::to_string(cli::FlagEp{pathEpResultSCML}),
      cli::to_string(cli::FlagAgencyId{"TEST"})};

  auto flagsSample{sample.asFlags()};
  flagsStr.insert(std::end(flagsStr), std::begin(flagsSample),
                  std::end(flagsSample));

  BOOST_TEST_MESSAGE("Running integration test with CLI args: "
                     << boost::algorithm::join(flagsStr, " "));
  BOOST_TEST_MESSAGE("Path to temporary test data: " << fx.pathTempdir);
  /* try { */
  /*   // parse README */
  /*   std::string readmeHeader; */
  /*   fs::ifstream ifs{CLIParserFixture::pathData / sample.pathSample /
   * "README"}; */
  /*   getline(ifs, readmeHeader); */
  /*   if (!readmeHeader.empty()) { */
  /*     BOOST_TEST_MESSAGE("Test purpose: " << readmeHeader); */
  /*     getline(ifs, readmeHeader); */
  /*     std::stringstream buffer; */
  /*     if (buffer << ifs.rdbuf()) { */
  /*       BOOST_TEST_MESSAGE("Test description and configuration:\n\n" */
  /*                          << buffer.str()); */
  /*     } */
  /*   } */
  /* } catch (fs::filesystem_error &e) { */
  /* } */

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
  fs::path pathEpExpectedSCML{sample.pathExpected};
  readEventParameters(pathEpExpectedSCML, epExpected);
  BOOST_TEST_REQUIRE(epExpected, "Failed to read file: " << pathEpExpectedSCML);

  eventParametersCmp(epResult, epExpected);
}

}  // namespace test
}  // namespace detect
}  // namespace Seiscomp
