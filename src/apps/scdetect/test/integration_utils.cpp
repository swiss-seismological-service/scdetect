#include "integration_utils.h"

#include <algorithm>
#include <ostream>
#include <sstream>
#include <vector>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/optional.hpp>
#include <boost/program_options.hpp>
#include <boost/test/unit_test.hpp>

#include <seiscomp/core/exceptions.h>

#include "../utils.h"

namespace fs = boost::filesystem;
namespace po = boost::program_options;
namespace utf = boost::unit_test;
namespace utf_tt = boost::test_tools;

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

template <typename TPtr, typename TFunc, typename TPred>
std::vector<TPtr> SortByPredicate(const TFunc &f, size_t n, const TPred &pred) {

  std::vector<TPtr> ret;
  for (size_t i = 0; i < n; ++i) {
    ret.push_back(f(i));
  }

  std::sort(std::begin(ret), std::end(ret), pred);
  return ret;
}

// TODO(damb): Define function return value with regards to `SortByPredicate()`
template <typename TPtr, typename TFunc>
std::vector<TPtr> SortByTime(const TFunc &f, size_t n) {
  return SortByPredicate<TPtr>(f, n, [](const TPtr &lhs, const TPtr &rhs) {
    return lhs->time().value() < rhs->time().value();
  });
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
void ArgFlag::set_arg(const std::string &arg) { arg_ = arg; }

BooleanFlag::BooleanFlag() : ArgFlag{"1"} {}
void BooleanFlag::Enable() { set_arg("1"); }
void BooleanFlag::Disable() { set_arg("0"); }

const std::string FlagDebug::flag() const { return "--debug"; }

const std::string FlagConsole::flag() const { return "--console"; }

const std::string FlagOffline::flag() const { return std::string{"--offline"}; }

const std::string FlagPlayback::flag() const {
  return std::string{"--playback"};
}

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

FlagConfigFile::FlagConfigFile(const std::string &fpath) : ArgFlag{fpath} {}
FlagConfigFile::FlagConfigFile(const fs::path &fpath)
    : ArgFlag{fpath.string()} {}
const std::string FlagConfigFile::flag() const { return "--config-file"; }

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
void EventParametersCmp(const DataModel::EventParametersCPtr &lhs,
                        const DataModel::EventParametersCPtr &rhs) {

  BOOST_TEST_CHECK(lhs->pickCount() == rhs->pickCount());
  BOOST_TEST_CHECK(lhs->originCount() == rhs->originCount());

  BOOST_TEST_CHECK(lhs->eventCount() == rhs->eventCount());

  BOOST_TEST_CHECK(lhs->amplitudeCount() == rhs->amplitudeCount());
  BOOST_TEST_CHECK(lhs->focalMechanismCount() == rhs->focalMechanismCount());

  // compare picks
  const auto lhs_picks{SortByTime<DataModel::PickCPtr>(
      [&lhs](size_t i) { return lhs->pick(i); }, lhs->pickCount())};
  const auto rhs_picks{SortByTime<DataModel::PickCPtr>(
      [&rhs](size_t i) { return rhs->pick(i); }, rhs->pickCount())};
  for (size_t i = 0; i < lhs_picks.size(); ++i) {
    DataModel::PickCPtr pick_result{lhs_picks.at(i)};
    DataModel::PickCPtr pick_expected{rhs_picks.at(i)};

    PickCmp(pick_result, pick_expected);
  }

  // compare origins
  const auto OriginPredicate = [](const DataModel::OriginCPtr &lhs,
                                  const DataModel::OriginCPtr &rhs) {
    // XXX(damb): Used to generate a pseudo total order
    const auto MinimumDistancePredicate = [](const DataModel::OriginCPtr &o) {
      return o->quality().minimumDistance();
    };
    return MinimumDistancePredicate(lhs) < MinimumDistancePredicate(rhs);
  };

  const auto lhs_origins{SortByPredicate<DataModel::OriginCPtr>(
      [&lhs](size_t i) { return lhs->origin(i); }, lhs->originCount(),
      OriginPredicate)};
  const auto rhs_origins{SortByPredicate<DataModel::OriginCPtr>(
      [&rhs](size_t i) { return rhs->origin(i); }, rhs->originCount(),
      OriginPredicate)};
  for (size_t i = 0; i < lhs_origins.size(); ++i) {
    DataModel::OriginCPtr origin_result{lhs_origins.at(i)};
    DataModel::OriginCPtr origin_expected{rhs_origins.at(i)};

    OriginCmp(origin_result, origin_expected);
  }
}

void PickCmp(const DataModel::PickCPtr &lhs, const DataModel::PickCPtr &rhs) {
  // compare attributes since the `creationInfo` attribute differs, anyway
  BOOST_TEST_CHECK(static_cast<double>(lhs->time().value()) ==
                   static_cast<double>(rhs->time().value()));
  BOOST_TEST_CHECK(std::string{lhs->waveformID()} ==
                   std::string{rhs->waveformID()});
  BOOST_TEST_CHECK(lhs->filterID() == rhs->filterID());
  BOOST_TEST_CHECK(lhs->methodID() == rhs->methodID());

  BOOST_TEST_CHECK(EqualOptional(
      lhs, rhs, [](DataModel::PickCPtr p) { return p->horizontalSlowness(); }));
  BOOST_TEST_CHECK(EqualOptional(
      lhs, rhs, [](DataModel::PickCPtr p) { return p->backazimuth(); }));

  BOOST_TEST_CHECK(lhs->slownessMethodID() == rhs->slownessMethodID());

  BOOST_TEST_CHECK(EqualOptional(
      lhs, rhs, [](DataModel::PickCPtr p) { return p->onset(); }));
  BOOST_TEST_CHECK(EqualOptional(
      lhs, rhs, [](DataModel::PickCPtr p) { return p->phaseHint(); }));
  BOOST_TEST_CHECK(EqualOptional(
      lhs, rhs, [](DataModel::PickCPtr p) { return p->polarity(); }));
  BOOST_TEST_CHECK(EqualOptional(
      lhs, rhs, [](DataModel::PickCPtr p) { return p->evaluationMode(); }));
  BOOST_TEST_CHECK(EqualOptional(
      lhs, rhs, [](DataModel::PickCPtr p) { return p->evaluationStatus(); }));

  BOOST_TEST_CHECK(EqualOptional(lhs, rhs, [](DataModel::PickCPtr p) {
    return p->creationInfo().agencyID();
  }));
}

void OriginCmp(const DataModel::OriginCPtr &lhs,
               const DataModel::OriginCPtr &rhs) {
  BOOST_TEST_CHECK(static_cast<double>(lhs->time().value()) ==
                   static_cast<double>(rhs->time().value()));
  BOOST_TEST_CHECK(lhs->latitude() == rhs->latitude());
  BOOST_TEST_CHECK(lhs->longitude() == rhs->longitude());

  BOOST_TEST_CHECK(EqualOptional(
      lhs, rhs, [](DataModel::OriginCPtr orig) { return orig->depth(); }));
  BOOST_TEST_CHECK(EqualOptional(
      lhs, rhs, [](DataModel::OriginCPtr orig) { return orig->depthType(); }));
  BOOST_TEST_CHECK(EqualOptional(
      lhs, rhs, [](DataModel::OriginCPtr orig) { return orig->timeFixed(); }));
  BOOST_TEST_CHECK(EqualOptional(lhs, rhs, [](DataModel::OriginCPtr orig) {
    return orig->epicenterFixed();
  }));

  BOOST_TEST_CHECK(lhs->referenceSystemID() == rhs->referenceSystemID());
  BOOST_TEST_CHECK(lhs->methodID() == rhs->methodID());
  BOOST_TEST_CHECK(lhs->earthModelID() == rhs->earthModelID());

  auto lhs_q{utils::make_smart<DataModel::OriginQuality>(lhs->quality())};
  auto rhs_q{utils::make_smart<DataModel::OriginQuality>(rhs->quality())};
  OriginQualityCmp(lhs_q, rhs_q);

  BOOST_TEST_CHECK(EqualOptional(lhs, rhs, [](DataModel::OriginCPtr orig) {
    return orig->uncertainty();
  }));
  BOOST_TEST_CHECK(EqualOptional(
      lhs, rhs, [](DataModel::OriginCPtr orig) { return orig->type(); }));
  BOOST_TEST_CHECK(EqualOptional(lhs, rhs, [](DataModel::OriginCPtr orig) {
    return orig->evaluationMode();
  }));
  BOOST_TEST_CHECK(EqualOptional(lhs, rhs, [](DataModel::OriginCPtr orig) {
    return orig->evaluationStatus();
  }));

  BOOST_TEST_CHECK(EqualOptional(lhs, rhs, [](DataModel::OriginCPtr orig) {
    return orig->creationInfo().agencyID();
  }));

  // compare arrivals
  BOOST_TEST_CHECK(lhs->arrivalCount() == rhs->arrivalCount());
  const auto PhaseCodePredicate = [](const DataModel::ArrivalCPtr &lhs,
                                     const DataModel::ArrivalCPtr &rhs) {
    return lhs->phase().code() < rhs->phase().code();
  };
  const auto lhs_arrivals{SortByPredicate<DataModel::ArrivalCPtr>(
      [&lhs](size_t i) { return lhs->arrival(i); }, lhs->arrivalCount(),
      PhaseCodePredicate)};
  const auto rhs_arrivals{SortByPredicate<DataModel::ArrivalCPtr>(
      [&rhs](size_t i) { return rhs->arrival(i); }, rhs->arrivalCount(),
      PhaseCodePredicate)};
  for (size_t j = 0; j < lhs_arrivals.size(); ++j) {
    DataModel::ArrivalCPtr arrival_result{lhs_arrivals.at(j)};
    DataModel::ArrivalCPtr arrival_expected{rhs_arrivals.at(j)};

    ArrivalCmp(arrival_result, arrival_expected);
  }

  // compare magnitudes
  BOOST_TEST_CHECK(lhs->magnitudeCount() == rhs->magnitudeCount());
  const auto MagnitudePredicate = [](const DataModel::MagnitudeCPtr &lhs,
                                     const DataModel::MagnitudeCPtr &rhs) {
    return lhs->magnitude().value() < rhs->magnitude().value() &&
           lhs->type() < rhs->type();
  };
  const auto lhs_mags{SortByPredicate<DataModel::MagnitudeCPtr>(
      [&lhs](size_t i) { return lhs->magnitude(i); }, lhs->magnitudeCount(),
      MagnitudePredicate)};
  const auto rhs_mags{SortByPredicate<DataModel::MagnitudeCPtr>(
      [&rhs](size_t i) { return rhs->magnitude(i); }, rhs->magnitudeCount(),
      MagnitudePredicate)};
  for (size_t j = 0; j < lhs_mags.size(); ++j) {
    DataModel::MagnitudeCPtr mag_result{lhs_mags.at(j)};
    DataModel::MagnitudeCPtr mag_expected{rhs_mags.at(j)};

    MagnitudeCmp(mag_result, mag_expected);
  }
}

void OriginQualityCmp(const DataModel::OriginQualityCPtr &lhs,
                      const DataModel::OriginQualityCPtr &rhs) {

  BOOST_TEST_CHECK(EqualOptional(lhs, rhs, [](DataModel::OriginQualityCPtr q) {
    return q->associatedPhaseCount();
  }));
  BOOST_TEST_CHECK(EqualOptional(lhs, rhs, [](DataModel::OriginQualityCPtr q) {
    return q->usedPhaseCount();
  }));
  BOOST_TEST_CHECK(EqualOptional(lhs, rhs, [](DataModel::OriginQualityCPtr q) {
    return q->associatedStationCount();
  }));
  BOOST_TEST_CHECK(EqualOptional(lhs, rhs, [](DataModel::OriginQualityCPtr q) {
    return q->usedStationCount();
  }));
  BOOST_TEST_CHECK(EqualOptional(lhs, rhs, [](DataModel::OriginQualityCPtr q) {
    return q->depthPhaseCount();
  }));
  const auto StandardErrorPredicate = [](DataModel::OriginQualityCPtr q) {
    return q->standardError();
  };
  BOOST_TEST_CHECK(*GetOptional(lhs, StandardErrorPredicate) ==
                       *GetOptional(rhs, StandardErrorPredicate),
                   utf_tt::tolerance(5.0e-3));
  const auto AzimuthalGapPredicate = [](DataModel::OriginQualityCPtr q) {
    return q->azimuthalGap();
  };
  BOOST_TEST_CHECK(GetOptional(lhs, AzimuthalGapPredicate).value_or(-1) ==
                   GetOptional(rhs, AzimuthalGapPredicate).value_or(-1));
  const auto SecondaryAzimuthalGapPredicate =
      [](DataModel::OriginQualityCPtr q) { return q->secondaryAzimuthalGap(); };
  BOOST_TEST_CHECK(
      GetOptional(lhs, SecondaryAzimuthalGapPredicate).value_or(-1) ==
      GetOptional(rhs, SecondaryAzimuthalGapPredicate).value_or(-1));
  BOOST_TEST_CHECK(lhs->groundTruthLevel() == rhs->groundTruthLevel());
  const auto MaximumDistancePredicate = [](DataModel::OriginQualityCPtr q) {
    return q->maximumDistance();
  };
  BOOST_TEST_CHECK(GetOptional(lhs, MaximumDistancePredicate).value_or(-1) ==
                   GetOptional(rhs, MaximumDistancePredicate).value_or(-1));
  const auto MinimumDistancePredicate = [](DataModel::OriginQualityCPtr q) {
    return q->minimumDistance();
  };
  BOOST_TEST_CHECK(GetOptional(lhs, MinimumDistancePredicate).value_or(-1) ==
                   GetOptional(rhs, MinimumDistancePredicate).value_or(-1));
  const auto MedianDistancePredicate = [](DataModel::OriginQualityCPtr q) {
    return q->medianDistance();
  };
  BOOST_TEST_CHECK(GetOptional(lhs, MedianDistancePredicate).value_or(-1) ==
                   GetOptional(rhs, MedianDistancePredicate).value_or(-1));
}

void ArrivalCmp(const DataModel::ArrivalCPtr &lhs,
                const DataModel::ArrivalCPtr &rhs) {
  BOOST_TEST_CHECK(lhs->phase().code() == rhs->phase().code());
  BOOST_TEST_CHECK(EqualOptional(
      lhs, rhs, [](DataModel::ArrivalCPtr a) { return a->timeCorrection(); }));
  BOOST_TEST_CHECK(EqualOptional(
      lhs, rhs, [](DataModel::ArrivalCPtr a) { return a->azimuth(); }));
  BOOST_TEST_CHECK(EqualOptional(
      lhs, rhs, [](DataModel::ArrivalCPtr a) { return a->distance(); }));
  BOOST_TEST_CHECK(EqualOptional(
      lhs, rhs, [](DataModel::ArrivalCPtr a) { return a->takeOffAngle(); }));
  BOOST_TEST_CHECK(EqualOptional(
      lhs, rhs, [](DataModel::ArrivalCPtr a) { return a->timeResidual(); }));
  BOOST_TEST_CHECK(EqualOptional(lhs, rhs, [](DataModel::ArrivalCPtr a) {
    return a->horizontalSlownessResidual();
  }));
  BOOST_TEST_CHECK(EqualOptional(
      lhs, rhs, [](DataModel::ArrivalCPtr a) { return a->timeUsed(); }));
  BOOST_TEST_CHECK(EqualOptional(lhs, rhs, [](DataModel::ArrivalCPtr a) {
    return a->horizontalSlownessUsed();
  }));
  BOOST_TEST_CHECK(EqualOptional(
      lhs, rhs, [](DataModel::ArrivalCPtr a) { return a->backazimuthUsed(); }));
  BOOST_TEST_CHECK(EqualOptional(
      lhs, rhs, [](DataModel::ArrivalCPtr a) { return a->weight(); }));
  BOOST_TEST_CHECK(lhs->earthModelID() == rhs->earthModelID());
  BOOST_TEST_CHECK(EqualOptional(
      lhs, rhs, [](DataModel::ArrivalCPtr a) { return a->preliminary(); }));

  BOOST_TEST_CHECK(EqualOptional(lhs, rhs, [](DataModel::ArrivalCPtr a) {
    return a->creationInfo().agencyID();
  }));
}

void MagnitudeCmp(const DataModel::MagnitudeCPtr &lhs,
                  const DataModel::MagnitudeCPtr &rhs) {
  BOOST_TEST_CHECK(lhs->magnitude() == rhs->magnitude());
  BOOST_TEST_CHECK(lhs->type() == rhs->type());
  BOOST_TEST_CHECK(lhs->originID() == rhs->originID());
  BOOST_TEST_CHECK(lhs->methodID() == rhs->methodID());

  BOOST_TEST_CHECK(EqualOptional(
      lhs, rhs, [](DataModel::MagnitudeCPtr m) { return m->stationCount(); }));
  BOOST_TEST_CHECK(EqualOptional(
      lhs, rhs, [](DataModel::MagnitudeCPtr m) { return m->azimuthalGap(); }));
  BOOST_TEST_CHECK(EqualOptional(lhs, rhs, [](DataModel::MagnitudeCPtr m) {
    return m->evaluationStatus();
  }));

  BOOST_TEST_CHECK(EqualOptional(lhs, rhs, [](DataModel::MagnitudeCPtr m) {
    return m->creationInfo().agencyID();
  }));
}

/* -------------------------------------------------------------------------- */
const std::string TempDirFixture::path_subdir{"scdetect"};
const int TempDirFixture::max_tries{5};

TempDirFixture::TempDirFixture() : path_tempdir{CreatePathUnique()} {
  CreateTempdir();
}

TempDirFixture::TempDirFixture(bool keep_tempdir)
    : path_tempdir{CreatePathUnique()}, keep_tempdir_{keep_tempdir} {
  CreateTempdir();
}

TempDirFixture::~TempDirFixture() {
  try {
    if (!keep_tempdir_) {
      fs::remove_all(path_tempdir);
    }
  } catch (fs::filesystem_error &e) {
  }
}

std::string TempDirFixture::path_tempdir_str() const {
  return path_tempdir.string();
}

const char *TempDirFixture::path_tempdir_cstr() const {
  return path_tempdir.c_str();
}

fs::path TempDirFixture::CreatePathUnique() {
  return fs::temp_directory_path() / TempDirFixture::path_subdir /
         fs::unique_path();
}

void TempDirFixture::CreateTempdir() {
  int tries{max_tries};
  while (fs::exists(path_tempdir)) {
    path_tempdir = CreatePathUnique();

    if (!(--tries)) {
      BOOST_FAIL("Failed to create temporary directory. Too many tries.");
    }
  }
  try {
    fs::create_directories(path_tempdir);
  } catch (fs::filesystem_error &e) {
    BOOST_FAIL("Failed to create temporary directory: " << e.what());
  }
}

/* ------------------------------------------------------------------------- */
fs::path CLIParserFixture::path_data{""};
bool CLIParserFixture::keep_tempdir{false};

CLIParserFixture::CLIParserFixture() {}
CLIParserFixture::~CLIParserFixture() {}

void CLIParserFixture::setup() {
  try {
    po::options_description desc;
    desc.add_options()("keep-tempfiles",
                       po::value<bool>(&keep_tempdir)->default_value(false),
                       "Keep temporary files from tests")(
        "path-data", po::value<fs::path>(&path_data),
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
      bool{fs::is_directory(path_data) && !fs::is_empty(path_data)},
      "Invalid path to test data directory:" << path_data);
  path_data = fs::absolute(path_data);
}

void CLIParserFixture::teardown() {}

} // namespace test
} // namespace detect
} // namespace Seiscomp
