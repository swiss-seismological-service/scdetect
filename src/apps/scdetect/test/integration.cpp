#include "integration.h"

#include <algorithm>
#include <ostream>
#include <sstream>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/test/unit_test.hpp>

namespace fs = boost::filesystem;

namespace Seiscomp {
namespace detect {
namespace test {

namespace cli {

std::string to_string(const Flag &flag) {
  std::ostringstream oss;
  oss << flag;
  return oss.str();
}

void Flag::to_string(std::ostream &os) const { os << flag(); }

std::ostream &operator<<(std::ostream &os, const Flag &flag) {
  flag.to_string(os);
  return os;
}

ArgFlag::ArgFlag(const std::string &arg) : arg_{arg} {}
void ArgFlag::to_string(std::ostream &os) const { os << flag() << "=" << arg_; }

const std::string FlagDebug::flag() const { return "--debug"; }

FlagConsole::FlagConsole() : ArgFlag{"1"} {}
const std::string FlagConsole::flag() const { return "--console"; }

FlagOffline::FlagOffline() : ArgFlag{"1"} {}
const std::string FlagOffline::flag() const { return std::string{"--offline"}; }

FlagTemplatesReload::FlagTemplatesReload() : ArgFlag{"1"} {}
const std::string FlagTemplatesReload::flag() const {
  return "--templates-reload";
}

FlagAgencyId::FlagAgencyId(const std::string &id) : ArgFlag{id} {}
const std::string FlagAgencyId::flag() const { return "--agencyID"; }

FlagAuthor::FlagAuthor(const std::string &name) : ArgFlag{name} {}
const std::string FlagAuthor::flag() const { return "--author"; }

FlagPlugins::FlagPlugins(const std::string &plugin) : ArgFlag{plugin} {
  // TODO(damb): Must not contain comma
}
FlagPlugins::FlagPlugins(const std::vector<std::string> &plugins)
    : ArgFlag{boost::algorithm::join(plugins, ",")} {}

const std::string FlagPlugins::flag() const { return "--plugins"; }

FlagEp::FlagEp(const std::string &fpath) : ArgFlag{fpath} {}
FlagEp ::FlagEp(const fs::path &fpath) : FlagEp{fpath.string()} {}
const std::string FlagEp::flag() const { return "--ep"; }

FlagDB::FlagDB(const std::string &uri) : ArgFlag{uri} {}
FlagDB ::FlagDB(const fs::path &fpath)
    : FlagDB{std::string{"sqlite3://" + fpath.string()}} {}
const std::string FlagDB::flag() const { return "--database"; }

FlagInventoryDB::FlagInventoryDB(const std::string &uri) : ArgFlag{uri} {}
FlagInventoryDB::FlagInventoryDB(const fs::path &fpath)
    : FlagInventoryDB{std::string{"file://" + fpath.string()}} {}
const std::string FlagInventoryDB::flag() const { return "--inventory-db"; }

FlagEventDB::FlagEventDB(const std::string &uri) : ArgFlag{std::string{uri}} {}
FlagEventDB::FlagEventDB(const fs::path &fpath)
    : FlagEventDB{std::string{"file://" + fpath.string()}} {}
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

FlagTemplatesJSON::FlagTemplatesJSON(const std::string &fpath)
    : ArgFlag{fpath} {}
FlagTemplatesJSON::FlagTemplatesJSON(const fs::path &fpath)
    : FlagTemplatesJSON{fpath.string()} {}
const std::string FlagTemplatesJSON::flag() const { return "--templates-json"; }

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
