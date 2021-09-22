#include "fixture.h"

#include <boost/program_options.hpp>
#include <boost/test/unit_test.hpp>

namespace fs = boost::filesystem;
namespace po = boost::program_options;
namespace utf = boost::unit_test;

namespace Seiscomp {
namespace detect {
namespace test {

fs::path CLIParserFixture::pathData{""};
bool CLIParserFixture::keepTempdir{false};

CLIParserFixture::CLIParserFixture() {}
CLIParserFixture::~CLIParserFixture() {}

void CLIParserFixture::setup() {
  try {
    po::options_description desc;
    desc.add_options()("keep-tempfiles",
                       po::value<bool>(&keepTempdir)->default_value(false),
                       "Keep temporary files from tests")(
        "path-data", po::value<fs::path>(&pathData),
        "Path to test data directory");

    po::positional_options_description pdesc;
    pdesc.add("path-data", -1);

    po::variables_map vm;
    po::store(po::command_line_parser(utf::framework::master_test_suite().argc,
                                      utf::framework::master_test_suite().argv)
                  .options(desc)
                  .positional(pdesc)
                  .run(),
              vm);
    po::notify(vm);
  } catch (std::exception &e) {
    BOOST_TEST_FAIL(e.what());
  }

  // validate
  BOOST_TEST_REQUIRE(
      bool{fs::is_directory(pathData) && !fs::is_empty(pathData)},
      "Invalid path to test data directory:" << pathData);
  pathData = fs::absolute(pathData);
}

void CLIParserFixture::teardown() {}

}  // namespace test
}  // namespace detect
}  // namespace Seiscomp
