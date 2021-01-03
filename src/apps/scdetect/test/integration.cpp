#include "integration.h"

#include <algorithm>

#include <boost/filesystem.hpp>
#include <boost/test/unit_test.hpp>

namespace fs = boost::filesystem;

namespace Seiscomp {
namespace detect {
namespace test {

namespace cli {

const std::string Flag::operator()() const { return flag(); }

ArgFlag::ArgFlag(const std::string &arg) : arg_{arg} {}
const std::string ArgFlag::operator()() const {
  return std::string{flag() + "=" + arg_};
}

const std::string FlagDebug::flag() const { return "--debug"; }

FlagOffline::FlagOffline() : ArgFlag{"1"} {}
const std::string FlagOffline::flag() const { return std::string{"--offline"}; }

FlagEp::FlagEp(const std::string &fpath) : ArgFlag{fpath} {}
const std::string FlagEp::flag() const { return "--ep"; }

FlagInventoryDB::FlagInventoryDB(const std::string &uri) : ArgFlag{uri} {}
const std::string FlagInventoryDB::flag() const { return "--inventory-db"; }

FlagEventDB::FlagEventDB(const std::string &uri) : ArgFlag{std::string{uri}} {}
const std::string FlagEventDB::flag() const { return "--event-db"; }

FlagRecordURL ::FlagRecordURL(const std::string &url) : ArgFlag{url} {}
const std::string FlagRecordURL::flag() const { return "--record-url"; }

FlagRecordStartTime::FlagRecordStartTime(const std::string &time_str)
    : ArgFlag{time_str} {}
const std::string FlagRecordStartTime::flag() const {
  return "--record-starttime";
}

FlagRecordEndTime::FlagRecordEndTime(const std::string &time_str)
    : ArgFlag{time_str} {}
const std::string FlagRecordEndTime::flag() const { return "--record-endtime"; }

FlagTemplateJSON::FlagTemplateJSON(const std::string &fpath) : ArgFlag{fpath} {}
const std::string FlagTemplateJSON::flag() const { return "--template-json"; }

} // namespace cli

const std::string PathData(const std::string &fname) {
  return fs::absolute(fs::path{"data"} / fname).string();
}

std::vector<char *>
StringsToCStrings(const std::vector<std::string> &v_strings) {
  std::vector<char *> v_cstrings{v_strings.size()};
  std::transform(
      v_strings.cbegin(), v_strings.cend(), v_cstrings.begin(),
      [](const std::string &str) { return const_cast<char *>(str.c_str()); });
  return v_cstrings;
}

/* --------------------------------------------------------------------------
 */
const std::string TempDirFixture::path_subdir{"scdetect"};

TempDirFixture::TempDirFixture() : path_tempdir{create_path_unique()} {
  int max_tries{5};
  while (fs::exists(path_tempdir)) {
    path_tempdir = create_path_unique();

    if (!(--max_tries)) {
      BOOST_FAIL("Failed to create temporary directory. Too many tries.");
    }
  }
  if (!fs::create_directories(path_tempdir)) {
    BOOST_FAIL(std::string{"Failed to create temporary directory: " +
                           path_tempdir.string()});
  }
}

TempDirFixture::~TempDirFixture() { fs::remove_all(path_tempdir); }

const std::string TempDirFixture::path_tempdir_str() const {
  return path_tempdir.string();
}

const char *TempDirFixture::path_tempdir_cstr() const {
  return path_tempdir.c_str();
}

const fs::path TempDirFixture::create_path_unique() {
  return fs::temp_directory_path() / TempDirFixture::path_subdir /
         fs::unique_path();
}

} // namespace test
} // namespace detect
} // namespace Seiscomp
