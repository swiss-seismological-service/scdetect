#include "integration.h"

#include <algorithm>
#include <ostream>
#include <sstream>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/optional.hpp>
#include <boost/test/unit_test.hpp>

#include <seiscomp/core/exceptions.h>
#include <seiscomp/datamodel/arrival.h>
#include <seiscomp/datamodel/magnitude.h>
#include <seiscomp/datamodel/origin.h>
#include <seiscomp/datamodel/pick.h>

namespace fs = boost::filesystem;
namespace utf = boost::unit_test;

namespace Seiscomp {
namespace detect {
namespace test {

namespace {

template <typename T, typename TFunc>
auto GetOptional(T obj, TFunc f) -> boost::optional<decltype(f(obj))> {
  try {
    return f(obj);
  } catch (Core::ValueException &e) {
    return boost::none;
  }
}

template <typename T, typename TFunc>
bool EqualOptional(const T &lhs, const T &rhs, TFunc f) {
  return GetOptional(lhs, f) == GetOptional(rhs, f);
}

} // namespace

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

/* ------------------------------------------------------------------------- */
void EventParametersCmp(DataModel::EventParametersCPtr lhs,
                        DataModel::EventParametersCPtr rhs) {

  BOOST_TEST_CHECK(lhs->pickCount() == rhs->pickCount());
  BOOST_TEST_CHECK(lhs->originCount() == rhs->originCount());

  BOOST_TEST_CHECK(lhs->eventCount() == rhs->eventCount());

  BOOST_TEST_CHECK(lhs->amplitudeCount() == rhs->amplitudeCount());
  BOOST_TEST_CHECK(lhs->focalMechanismCount() == rhs->focalMechanismCount());

  // compare picks
  for (size_t i = 0; i < lhs->pickCount(); ++i) {
    DataModel::PickCPtr pick_result{lhs->pick(i)};
    DataModel::PickCPtr pick_expected{rhs->pick(i)};

    // compare attributes since the `creationInfo` attribute differs, anyway

    BOOST_TEST_CHECK(pick_result->time().value().iso() ==
                     pick_expected->time().value().iso());
    BOOST_TEST_CHECK(std::string{pick_result->waveformID()} ==
                     std::string{pick_expected->waveformID()});
    BOOST_TEST_CHECK(pick_result->filterID() == pick_expected->filterID());
    BOOST_TEST_CHECK(pick_result->methodID() == pick_expected->methodID());

    BOOST_TEST_CHECK(
        EqualOptional(pick_result, pick_expected, [](DataModel::PickCPtr p) {
          return p->horizontalSlowness();
        }));
    BOOST_TEST_CHECK(
        EqualOptional(pick_result, pick_expected,
                      [](DataModel::PickCPtr p) { return p->backazimuth(); }));

    BOOST_TEST_CHECK(pick_result->slownessMethodID() ==
                     pick_expected->slownessMethodID());

    BOOST_TEST_CHECK(
        EqualOptional(pick_result, pick_expected,
                      [](DataModel::PickCPtr p) { return p->onset(); }));
    BOOST_TEST_CHECK(
        EqualOptional(pick_result, pick_expected,
                      [](DataModel::PickCPtr p) { return p->phaseHint(); }));
    BOOST_TEST_CHECK(
        EqualOptional(pick_result, pick_expected,
                      [](DataModel::PickCPtr p) { return p->polarity(); }));
    BOOST_TEST_CHECK(
        EqualOptional(pick_result, pick_expected, [](DataModel::PickCPtr p) {
          return p->evaluationMode();
        }));
    BOOST_TEST_CHECK(
        EqualOptional(pick_result, pick_expected, [](DataModel::PickCPtr p) {
          return p->evaluationStatus();
        }));

    BOOST_TEST_CHECK(
        EqualOptional(pick_result, pick_expected, [](DataModel::PickCPtr p) {
          return p->creationInfo().agencyID();
        }));
  }

  // compare origins
  for (size_t i = 0; i < lhs->originCount(); ++i) {
    DataModel::OriginCPtr origin_result{lhs->origin(i)};
    DataModel::OriginCPtr origin_expected{rhs->origin(i)};

    BOOST_TEST_CHECK(origin_result->time().value().iso() ==
                     origin_expected->time().value().iso());
    BOOST_TEST_CHECK(origin_result->latitude() == origin_expected->latitude());
    BOOST_TEST_CHECK(origin_result->longitude() ==
                     origin_expected->longitude());

    BOOST_TEST_CHECK(EqualOptional(
        origin_result, origin_expected,
        [](DataModel::OriginCPtr orig) { return orig->depth(); }));
    BOOST_TEST_CHECK(EqualOptional(
        origin_result, origin_expected,
        [](DataModel::OriginCPtr orig) { return orig->depthType(); }));
    BOOST_TEST_CHECK(EqualOptional(
        origin_result, origin_expected,
        [](DataModel::OriginCPtr orig) { return orig->timeFixed(); }));
    BOOST_TEST_CHECK(EqualOptional(
        origin_result, origin_expected,
        [](DataModel::OriginCPtr orig) { return orig->epicenterFixed(); }));

    BOOST_TEST_CHECK(origin_result->referenceSystemID() ==
                     origin_expected->referenceSystemID());
    BOOST_TEST_CHECK(origin_result->methodID() == origin_expected->methodID());
    BOOST_TEST_CHECK(origin_result->earthModelID() ==
                     origin_expected->earthModelID());

    BOOST_TEST_CHECK(EqualOptional(
        origin_result, origin_expected,
        [](DataModel::OriginCPtr orig) { return orig->quality(); }));
    BOOST_TEST_CHECK(EqualOptional(
        origin_result, origin_expected,
        [](DataModel::OriginCPtr orig) { return orig->uncertainty(); }));
    BOOST_TEST_CHECK(
        EqualOptional(origin_result, origin_expected,
                      [](DataModel::OriginCPtr orig) { return orig->type(); }));
    BOOST_TEST_CHECK(EqualOptional(
        origin_result, origin_expected,
        [](DataModel::OriginCPtr orig) { return orig->evaluationMode(); }));
    BOOST_TEST_CHECK(EqualOptional(
        origin_result, origin_expected,
        [](DataModel::OriginCPtr orig) { return orig->evaluationStatus(); }));

    BOOST_TEST_CHECK(EqualOptional(origin_result, origin_expected,
                                   [](DataModel::OriginCPtr orig) {
                                     return orig->creationInfo().agencyID();
                                   }));

    // compare arrivals
    for (size_t j = 0; j < origin_result->arrivalCount(); ++j) {
      DataModel::ArrivalCPtr arrival_result{origin_result->arrival(j)};
      DataModel::ArrivalCPtr arrival_expected{origin_expected->arrival(j)};

      BOOST_TEST_CHECK(arrival_result->phase().code() ==
                       arrival_expected->phase().code());

      BOOST_TEST_CHECK(EqualOptional(
          arrival_result, arrival_expected,
          [](DataModel::ArrivalCPtr a) { return a->timeCorrection(); }));
      BOOST_TEST_CHECK(
          EqualOptional(arrival_result, arrival_expected,
                        [](DataModel::ArrivalCPtr a) { return a->azimuth(); }));
      BOOST_TEST_CHECK(EqualOptional(
          arrival_result, arrival_expected,
          [](DataModel::ArrivalCPtr a) { return a->distance(); }));
      BOOST_TEST_CHECK(EqualOptional(
          arrival_result, arrival_expected,
          [](DataModel::ArrivalCPtr a) { return a->takeOffAngle(); }));
      BOOST_TEST_CHECK(EqualOptional(
          arrival_result, arrival_expected,
          [](DataModel::ArrivalCPtr a) { return a->timeResidual(); }));
      BOOST_TEST_CHECK(EqualOptional(arrival_result, arrival_expected,
                                     [](DataModel::ArrivalCPtr a) {
                                       return a->horizontalSlownessResidual();
                                     }));
      BOOST_TEST_CHECK(EqualOptional(
          arrival_result, arrival_expected,
          [](DataModel::ArrivalCPtr a) { return a->timeUsed(); }));
      BOOST_TEST_CHECK(EqualOptional(arrival_result, arrival_expected,
                                     [](DataModel::ArrivalCPtr a) {
                                       return a->horizontalSlownessUsed();
                                     }));
      BOOST_TEST_CHECK(EqualOptional(
          arrival_result, arrival_expected,
          [](DataModel::ArrivalCPtr a) { return a->backazimuthUsed(); }));
      BOOST_TEST_CHECK(
          EqualOptional(arrival_result, arrival_expected,
                        [](DataModel::ArrivalCPtr a) { return a->weight(); }));
      BOOST_TEST_CHECK(arrival_result->earthModelID() ==
                       arrival_expected->earthModelID());
      BOOST_TEST_CHECK(EqualOptional(
          arrival_result, arrival_expected,
          [](DataModel::ArrivalCPtr a) { return a->preliminary(); }));

      BOOST_TEST_CHECK(EqualOptional(arrival_result, arrival_expected,
                                     [](DataModel::ArrivalCPtr a) {
                                       return a->creationInfo().agencyID();
                                     }));
    }

    // compare magnitudes
    for (size_t j = 0; j < origin_result->magnitudeCount(); ++j) {
      DataModel::MagnitudeCPtr mag_result{origin_result->magnitude(j)};
      DataModel::MagnitudeCPtr mag_expected{origin_result->magnitude(j)};

      BOOST_TEST_CHECK(mag_result->magnitude() == mag_expected->magnitude());
      BOOST_TEST_CHECK(mag_result->type() == mag_expected->type());
      BOOST_TEST_CHECK(mag_result->originID() == mag_expected->originID());
      BOOST_TEST_CHECK(mag_result->methodID() == mag_expected->methodID());

      BOOST_TEST_CHECK(EqualOptional(
          mag_result, mag_expected,
          [](DataModel::MagnitudeCPtr m) { return m->stationCount(); }));
      BOOST_TEST_CHECK(EqualOptional(
          mag_result, mag_expected,
          [](DataModel::MagnitudeCPtr m) { return m->azimuthalGap(); }));
      BOOST_TEST_CHECK(EqualOptional(
          mag_result, mag_expected,
          [](DataModel::MagnitudeCPtr m) { return m->evaluationStatus(); }));

      BOOST_TEST_CHECK(EqualOptional(mag_result, mag_expected,
                                     [](DataModel::MagnitudeCPtr m) {
                                       return m->creationInfo().agencyID();
                                     }));
    }
  }
}

/* -------------------------------------------------------------------------- */
const std::string TempDirFixture::path_subdir{"scdetect"};

TempDirFixture::TempDirFixture() : path_tempdir{create_path_unique()} {
  int max_tries{5};
  while (fs::exists(path_tempdir)) {
    path_tempdir = create_path_unique();

    if (!(--max_tries)) {
      BOOST_FAIL("Failed to create temporary directory. Too many tries.");
    }
  }
  try {
    fs::create_directories(path_tempdir);
  } catch (fs::filesystem_error &e) {
    BOOST_FAIL("Failed to create temporary directory: " << e.what());
  }
}

TempDirFixture::~TempDirFixture() {
  try {
    fs::remove_all(path_tempdir);
  } catch (fs::filesystem_error &e) {
  }
}

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

/* ------------------------------------------------------------------------- */
fs::path CLIPathData::path{""};

CLIPathData::CLIPathData() {
  BOOST_TEST_REQUIRE(utf::framework::master_test_suite().argc == 2);
  BOOST_TEST_REQUIRE(utf::framework::master_test_suite().argv[2]);
}

void CLIPathData::setup() {
  fs::path p{utf::framework::master_test_suite().argv[1]};
  bool path_valid{fs::is_directory(p) && !fs::is_empty(p)};
  BOOST_TEST_REQUIRE(path_valid, "Invalid path to test data directory:" << p);
  path = fs::absolute(p);
}

void CLIPathData::teardown() {}

} // namespace test
} // namespace detect
} // namespace Seiscomp
