#include "integration_utils.h"

#include <seiscomp/core/exceptions.h>

#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/optional.hpp>
#include <boost/program_options.hpp>
#include <boost/test/unit_test.hpp>
#include <ostream>
#include <sstream>
#include <vector>

#include "../settings.h"
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
auto getOptional(T obj, TFunc f) -> boost::optional<decltype(f(obj))> {
  try {
    return f(obj);
  } catch (Core::ValueException &e) {
    return boost::none;
  }
}

template <typename T, typename TFunc>
bool equalOptional(const T &lhs, const T &rhs, TFunc f) {
  return getOptional(lhs, f) == getOptional(rhs, f);
}

template <typename TPtr, typename TFunc, typename TPred>
std::vector<TPtr> sortByPredicate(const TFunc &f, size_t n, const TPred &pred) {
  std::vector<TPtr> ret;
  for (size_t i = 0; i < n; ++i) {
    ret.push_back(f(i));
  }

  std::sort(std::begin(ret), std::end(ret), pred);
  return ret;
}

// TODO(damb): Define function return value with regards to `sortByPredicate()`
template <typename TPtr, typename TFunc>
std::vector<TPtr> sortByTime(const TFunc &f, size_t n) {
  return sortByPredicate<TPtr>(f, n, [](const TPtr &lhs, const TPtr &rhs) {
    return lhs->time().value() < rhs->time().value();
  });
}

}  // namespace

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

ArgFlag::ArgFlag(const std::string &arg) : _arg{arg} {}
void ArgFlag::to_string(std::ostream &os) const { os << flag() << "=" << _arg; }
void ArgFlag::setArg(const std::string &arg) { _arg = arg; }

BooleanFlag::BooleanFlag() : ArgFlag{"1"} {}
BooleanFlag::BooleanFlag(bool enabled) : ArgFlag{enabled ? "1" : "0"} {}
void BooleanFlag::enable() { setArg("1"); }
void BooleanFlag::disable() { setArg("0"); }

const std::string FlagDebug::flag() const { return "--debug"; }

const std::string FlagConsole::flag() const { return "--console"; }

FlagAmplitudesForce::FlagAmplitudesForce(bool enabled) : BooleanFlag{enabled} {}
const std::string FlagAmplitudesForce::flag() const {
  return "--amplitudes-force";
}

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
    : ArgFlag{boost::algorithm::join(plugins, settings::kConfigListSep)} {}

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

FlagRecordStartTime::FlagRecordStartTime(const std::string &timeStr)
    : ArgFlag{timeStr} {}
const std::string FlagRecordStartTime::flag() const {
  return "--record-starttime";
}

FlagRecordEndTime::FlagRecordEndTime(const std::string &timeStr)
    : ArgFlag{timeStr} {}
const std::string FlagRecordEndTime::flag() const { return "--record-endtime"; }

FlagTemplatesJSON::FlagTemplatesJSON(const std::string &fpath)
    : ArgFlag{fpath} {}
FlagTemplatesJSON::FlagTemplatesJSON(const fs::path &fpath)
    : FlagTemplatesJSON{fpath.string()} {}
const std::string FlagTemplatesJSON::flag() const { return "--templates-json"; }

}  // namespace cli

/* ------------------------------------------------------------------------- */
void eventParametersCmp(const DataModel::EventParametersCPtr &lhs,
                        const DataModel::EventParametersCPtr &rhs) {
  BOOST_TEST_CHECK(lhs->pickCount() == rhs->pickCount());
  BOOST_TEST_CHECK(lhs->originCount() == rhs->originCount());

  BOOST_TEST_CHECK(lhs->eventCount() == rhs->eventCount());

  BOOST_TEST_CHECK(lhs->amplitudeCount() == rhs->amplitudeCount());
  BOOST_TEST_CHECK(lhs->focalMechanismCount() == rhs->focalMechanismCount());

  // compare picks
  const auto lhsPicks{sortByTime<DataModel::PickCPtr>(
      [&lhs](size_t i) { return lhs->pick(i); }, lhs->pickCount())};
  const auto rhsPicks{sortByTime<DataModel::PickCPtr>(
      [&rhs](size_t i) { return rhs->pick(i); }, rhs->pickCount())};
  for (size_t i = 0; i < lhsPicks.size(); ++i) {
    DataModel::PickCPtr pickResult{lhsPicks.at(i)};
    DataModel::PickCPtr pickExpected{rhsPicks.at(i)};

    pickCmp(pickResult, pickExpected);
  }

  // compare origins
  const auto originPredicate = [](const DataModel::OriginCPtr &lhs,
                                  const DataModel::OriginCPtr &rhs) {
    // XXX(damb): Used to generate a pseudo total order
    const auto minimumDistancePredicate = [](const DataModel::OriginCPtr &o) {
      return o->quality().minimumDistance();
    };
    return minimumDistancePredicate(lhs) < minimumDistancePredicate(rhs);
  };

  const auto lhsOrigins{sortByPredicate<DataModel::OriginCPtr>(
      [&lhs](size_t i) { return lhs->origin(i); }, lhs->originCount(),
      originPredicate)};
  const auto rhsOrigins{sortByPredicate<DataModel::OriginCPtr>(
      [&rhs](size_t i) { return rhs->origin(i); }, rhs->originCount(),
      originPredicate)};
  for (size_t i = 0; i < lhsOrigins.size(); ++i) {
    DataModel::OriginCPtr originResult{lhsOrigins.at(i)};
    DataModel::OriginCPtr originExpected{rhsOrigins.at(i)};

    originCmp(originResult, originExpected);
  }
}

void pickCmp(const DataModel::PickCPtr &lhs, const DataModel::PickCPtr &rhs) {
  // compare attributes since the `creationInfo` attribute differs, anyway
  BOOST_TEST_CHECK(static_cast<double>(lhs->time().value()) ==
                   static_cast<double>(rhs->time().value()));
  BOOST_TEST_CHECK(std::string{lhs->waveformID()} ==
                   std::string{rhs->waveformID()});
  BOOST_TEST_CHECK(lhs->filterID() == rhs->filterID());
  BOOST_TEST_CHECK(lhs->methodID() == rhs->methodID());

  BOOST_TEST_CHECK(equalOptional(
      lhs, rhs, [](DataModel::PickCPtr p) { return p->horizontalSlowness(); }));
  BOOST_TEST_CHECK(equalOptional(
      lhs, rhs, [](DataModel::PickCPtr p) { return p->backazimuth(); }));

  BOOST_TEST_CHECK(lhs->slownessMethodID() == rhs->slownessMethodID());

  BOOST_TEST_CHECK(equalOptional(
      lhs, rhs, [](DataModel::PickCPtr p) { return p->onset(); }));
  BOOST_TEST_CHECK(equalOptional(
      lhs, rhs, [](DataModel::PickCPtr p) { return p->phaseHint(); }));
  BOOST_TEST_CHECK(equalOptional(
      lhs, rhs, [](DataModel::PickCPtr p) { return p->polarity(); }));
  BOOST_TEST_CHECK(equalOptional(
      lhs, rhs, [](DataModel::PickCPtr p) { return p->evaluationMode(); }));
  BOOST_TEST_CHECK(equalOptional(
      lhs, rhs, [](DataModel::PickCPtr p) { return p->evaluationStatus(); }));

  BOOST_TEST_CHECK(equalOptional(lhs, rhs, [](DataModel::PickCPtr p) {
    return p->creationInfo().agencyID();
  }));
}

void originCmp(const DataModel::OriginCPtr &lhs,
               const DataModel::OriginCPtr &rhs) {
  BOOST_TEST_CHECK(static_cast<double>(lhs->time().value()) ==
                   static_cast<double>(rhs->time().value()));
  BOOST_TEST_CHECK(lhs->latitude() == rhs->latitude());
  BOOST_TEST_CHECK(lhs->longitude() == rhs->longitude());

  BOOST_TEST_CHECK(equalOptional(
      lhs, rhs, [](DataModel::OriginCPtr orig) { return orig->depth(); }));
  BOOST_TEST_CHECK(equalOptional(
      lhs, rhs, [](DataModel::OriginCPtr orig) { return orig->depthType(); }));
  BOOST_TEST_CHECK(equalOptional(
      lhs, rhs, [](DataModel::OriginCPtr orig) { return orig->timeFixed(); }));
  BOOST_TEST_CHECK(equalOptional(lhs, rhs, [](DataModel::OriginCPtr orig) {
    return orig->epicenterFixed();
  }));

  BOOST_TEST_CHECK(lhs->referenceSystemID() == rhs->referenceSystemID());
  BOOST_TEST_CHECK(lhs->methodID() == rhs->methodID());
  BOOST_TEST_CHECK(lhs->earthModelID() == rhs->earthModelID());

  auto lhsOriginQuality{
      utils::make_smart<DataModel::OriginQuality>(lhs->quality())};
  auto rhsOriginQuality{
      utils::make_smart<DataModel::OriginQuality>(rhs->quality())};
  originQualityCmp(lhsOriginQuality, rhsOriginQuality);

  BOOST_TEST_CHECK(equalOptional(lhs, rhs, [](DataModel::OriginCPtr orig) {
    return orig->uncertainty();
  }));
  BOOST_TEST_CHECK(equalOptional(
      lhs, rhs, [](DataModel::OriginCPtr orig) { return orig->type(); }));
  BOOST_TEST_CHECK(equalOptional(lhs, rhs, [](DataModel::OriginCPtr orig) {
    return orig->evaluationMode();
  }));
  BOOST_TEST_CHECK(equalOptional(lhs, rhs, [](DataModel::OriginCPtr orig) {
    return orig->evaluationStatus();
  }));

  BOOST_TEST_CHECK(equalOptional(lhs, rhs, [](DataModel::OriginCPtr orig) {
    return orig->creationInfo().agencyID();
  }));

  // compare arrivals
  BOOST_TEST_CHECK(lhs->arrivalCount() == rhs->arrivalCount());
  const auto phaseCodePredicate = [](const DataModel::ArrivalCPtr &lhs,
                                     const DataModel::ArrivalCPtr &rhs) {
    return lhs->phase().code() < rhs->phase().code();
  };
  const auto lhsArrivals{sortByPredicate<DataModel::ArrivalCPtr>(
      [&lhs](size_t i) { return lhs->arrival(i); }, lhs->arrivalCount(),
      phaseCodePredicate)};
  const auto rhsArrivals{sortByPredicate<DataModel::ArrivalCPtr>(
      [&rhs](size_t i) { return rhs->arrival(i); }, rhs->arrivalCount(),
      phaseCodePredicate)};
  for (size_t j = 0; j < lhsArrivals.size(); ++j) {
    DataModel::ArrivalCPtr arrivalResult{lhsArrivals.at(j)};
    DataModel::ArrivalCPtr arrivalExpected{rhsArrivals.at(j)};

    arrivalCmp(arrivalResult, arrivalExpected);
  }

  // compare magnitudes
  BOOST_TEST_CHECK(lhs->magnitudeCount() == rhs->magnitudeCount());
  const auto magnitudePredicate = [](const DataModel::MagnitudeCPtr &lhs,
                                     const DataModel::MagnitudeCPtr &rhs) {
    return lhs->magnitude().value() < rhs->magnitude().value() &&
           lhs->type() < rhs->type();
  };
  const auto lhsMags{sortByPredicate<DataModel::MagnitudeCPtr>(
      [&lhs](size_t i) { return lhs->magnitude(i); }, lhs->magnitudeCount(),
      magnitudePredicate)};
  const auto rhsMags{sortByPredicate<DataModel::MagnitudeCPtr>(
      [&rhs](size_t i) { return rhs->magnitude(i); }, rhs->magnitudeCount(),
      magnitudePredicate)};
  for (size_t j = 0; j < lhsMags.size(); ++j) {
    DataModel::MagnitudeCPtr magResult{lhsMags.at(j)};
    DataModel::MagnitudeCPtr magExpected{rhsMags.at(j)};

    magnitudeCmp(magResult, magExpected);
  }

  // compare comments
  BOOST_TEST_CHECK(lhs->commentCount() == rhs->commentCount());
  const auto commentPredicate = [](const DataModel::CommentCPtr &lhs,
                                   const DataModel::CommentCPtr &rhs) {
    return lhs->id() < rhs->id() && lhs->text() < rhs->text();
  };
  const auto lhsComments{sortByPredicate<DataModel::CommentCPtr>(
      [&lhs](size_t i) { return lhs->comment(i); }, lhs->commentCount(),
      commentPredicate)};
  const auto rhsComments{sortByPredicate<DataModel::CommentCPtr>(
      [&rhs](size_t i) { return rhs->comment(i); }, rhs->commentCount(),
      commentPredicate)};
  for (size_t j = 0; j < lhsComments.size(); ++j) {
    DataModel::CommentCPtr commentResult{lhsComments.at(j)};
    DataModel::CommentCPtr commentExpected{rhsComments.at(j)};

    commentCmp(commentResult, commentExpected);
  }
}

void originQualityCmp(const DataModel::OriginQualityCPtr &lhs,
                      const DataModel::OriginQualityCPtr &rhs) {
  BOOST_TEST_CHECK(equalOptional(lhs, rhs, [](DataModel::OriginQualityCPtr q) {
    return q->associatedPhaseCount();
  }));
  BOOST_TEST_CHECK(equalOptional(lhs, rhs, [](DataModel::OriginQualityCPtr q) {
    return q->usedPhaseCount();
  }));
  BOOST_TEST_CHECK(equalOptional(lhs, rhs, [](DataModel::OriginQualityCPtr q) {
    return q->associatedStationCount();
  }));
  BOOST_TEST_CHECK(equalOptional(lhs, rhs, [](DataModel::OriginQualityCPtr q) {
    return q->usedStationCount();
  }));
  BOOST_TEST_CHECK(equalOptional(lhs, rhs, [](DataModel::OriginQualityCPtr q) {
    return q->depthPhaseCount();
  }));
  const auto standardErrorPredicate = [](DataModel::OriginQualityCPtr q) {
    return q->standardError();
  };
  BOOST_TEST_CHECK(*getOptional(lhs, standardErrorPredicate) ==
                       *getOptional(rhs, standardErrorPredicate),
                   utf_tt::tolerance(5.0e-3));
  const auto azimuthalGapPredicate = [](DataModel::OriginQualityCPtr q) {
    return q->azimuthalGap();
  };
  BOOST_TEST_CHECK(getOptional(lhs, azimuthalGapPredicate).value_or(-1) ==
                   getOptional(rhs, azimuthalGapPredicate).value_or(-1));
  const auto secondaryAzimuthalGapPredicate =
      [](DataModel::OriginQualityCPtr q) { return q->secondaryAzimuthalGap(); };
  BOOST_TEST_CHECK(
      getOptional(lhs, secondaryAzimuthalGapPredicate).value_or(-1) ==
      getOptional(rhs, secondaryAzimuthalGapPredicate).value_or(-1));
  BOOST_TEST_CHECK(lhs->groundTruthLevel() == rhs->groundTruthLevel());
  const auto maximumDistancePredicate = [](DataModel::OriginQualityCPtr q) {
    return q->maximumDistance();
  };
  BOOST_TEST_CHECK(getOptional(lhs, maximumDistancePredicate).value_or(-1) ==
                   getOptional(rhs, maximumDistancePredicate).value_or(-1));
  const auto minimumDistancePredicate = [](DataModel::OriginQualityCPtr q) {
    return q->minimumDistance();
  };
  BOOST_TEST_CHECK(getOptional(lhs, minimumDistancePredicate).value_or(-1) ==
                   getOptional(rhs, minimumDistancePredicate).value_or(-1));
  const auto medianDistancePredicate = [](DataModel::OriginQualityCPtr q) {
    return q->medianDistance();
  };
  BOOST_TEST_CHECK(getOptional(lhs, medianDistancePredicate).value_or(-1) ==
                   getOptional(rhs, medianDistancePredicate).value_or(-1));
}

void arrivalCmp(const DataModel::ArrivalCPtr &lhs,
                const DataModel::ArrivalCPtr &rhs) {
  BOOST_TEST_CHECK(lhs->phase().code() == rhs->phase().code());
  BOOST_TEST_CHECK(equalOptional(
      lhs, rhs, [](DataModel::ArrivalCPtr a) { return a->timeCorrection(); }));
  BOOST_TEST_CHECK(equalOptional(
      lhs, rhs, [](DataModel::ArrivalCPtr a) { return a->azimuth(); }));
  BOOST_TEST_CHECK(equalOptional(
      lhs, rhs, [](DataModel::ArrivalCPtr a) { return a->distance(); }));
  BOOST_TEST_CHECK(equalOptional(
      lhs, rhs, [](DataModel::ArrivalCPtr a) { return a->takeOffAngle(); }));
  BOOST_TEST_CHECK(equalOptional(
      lhs, rhs, [](DataModel::ArrivalCPtr a) { return a->timeResidual(); }));
  BOOST_TEST_CHECK(equalOptional(lhs, rhs, [](DataModel::ArrivalCPtr a) {
    return a->horizontalSlownessResidual();
  }));
  BOOST_TEST_CHECK(equalOptional(
      lhs, rhs, [](DataModel::ArrivalCPtr a) { return a->timeUsed(); }));
  BOOST_TEST_CHECK(equalOptional(lhs, rhs, [](DataModel::ArrivalCPtr a) {
    return a->horizontalSlownessUsed();
  }));
  BOOST_TEST_CHECK(equalOptional(
      lhs, rhs, [](DataModel::ArrivalCPtr a) { return a->backazimuthUsed(); }));
  BOOST_TEST_CHECK(equalOptional(
      lhs, rhs, [](DataModel::ArrivalCPtr a) { return a->weight(); }));
  BOOST_TEST_CHECK(lhs->earthModelID() == rhs->earthModelID());
  BOOST_TEST_CHECK(equalOptional(
      lhs, rhs, [](DataModel::ArrivalCPtr a) { return a->preliminary(); }));

  BOOST_TEST_CHECK(equalOptional(lhs, rhs, [](DataModel::ArrivalCPtr a) {
    return a->creationInfo().agencyID();
  }));
}

void magnitudeCmp(const DataModel::MagnitudeCPtr &lhs,
                  const DataModel::MagnitudeCPtr &rhs) {
  BOOST_TEST_CHECK(lhs->magnitude() == rhs->magnitude());
  BOOST_TEST_CHECK(lhs->type() == rhs->type());
  BOOST_TEST_CHECK(lhs->originID() == rhs->originID());
  BOOST_TEST_CHECK(lhs->methodID() == rhs->methodID());

  BOOST_TEST_CHECK(equalOptional(
      lhs, rhs, [](DataModel::MagnitudeCPtr m) { return m->stationCount(); }));
  BOOST_TEST_CHECK(equalOptional(
      lhs, rhs, [](DataModel::MagnitudeCPtr m) { return m->azimuthalGap(); }));
  BOOST_TEST_CHECK(equalOptional(lhs, rhs, [](DataModel::MagnitudeCPtr m) {
    return m->evaluationStatus();
  }));

  BOOST_TEST_CHECK(equalOptional(lhs, rhs, [](DataModel::MagnitudeCPtr m) {
    return m->creationInfo().agencyID();
  }));
}

void commentCmp(const DataModel::CommentCPtr &lhs,
                const DataModel::CommentCPtr &rhs) {
  BOOST_TEST_CHECK(lhs->id() == rhs->id());
  BOOST_TEST_CHECK(lhs->text() == rhs->text());

  BOOST_TEST_CHECK(equalOptional(lhs, rhs, [](DataModel::CommentCPtr c) {
    return static_cast<double>(c->start());
  }));
  BOOST_TEST_CHECK(equalOptional(lhs, rhs, [](DataModel::CommentCPtr c) {
    return static_cast<double>(c->end());
  }));
}

/* -------------------------------------------------------------------------- */
const std::string TempDirFixture::_pathSubDir{"scdetect"};
const int TempDirFixture::_maxTries{5};

TempDirFixture::TempDirFixture() : pathTempdir{createPathUnique()} {
  createTempdir();
}

TempDirFixture::TempDirFixture(bool keepTempdir)
    : pathTempdir{createPathUnique()}, _keepTempdir{keepTempdir} {
  createTempdir();
}

TempDirFixture::~TempDirFixture() {
  try {
    if (!_keepTempdir) {
      fs::remove_all(pathTempdir);
    }
  } catch (fs::filesystem_error &e) {
  }
}

std::string TempDirFixture::pathTempdirStr() const {
  return pathTempdir.string();
}

const char *TempDirFixture::pathTempdirCStr() const {
  return pathTempdir.c_str();
}

fs::path TempDirFixture::createPathUnique() {
  return fs::temp_directory_path() / TempDirFixture::_pathSubDir /
         fs::unique_path();
}

void TempDirFixture::createTempdir() {
  int tries{_maxTries};
  while (fs::exists(pathTempdir)) {
    pathTempdir = createPathUnique();

    if (!(--tries)) {
      BOOST_FAIL("Failed to create temporary directory. Too many tries.");
    }
  }
  try {
    fs::create_directories(pathTempdir);
  } catch (fs::filesystem_error &e) {
    BOOST_FAIL("Failed to create temporary directory: " << e.what());
  }
}

/* ------------------------------------------------------------------------- */
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
