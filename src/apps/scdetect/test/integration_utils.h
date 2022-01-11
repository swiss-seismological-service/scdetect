#ifndef SCDETECT_APPS_SCDETECT_TEST_INTEGRATION_UTILS_H_
#define SCDETECT_APPS_SCDETECT_TEST_INTEGRATION_UTILS_H_

#include <seiscomp/datamodel/amplitude.h>
#include <seiscomp/datamodel/arrival.h>
#include <seiscomp/datamodel/comment.h>
#include <seiscomp/datamodel/eventparameters.h>
#include <seiscomp/datamodel/magnitude.h>
#include <seiscomp/datamodel/origin.h>
#include <seiscomp/datamodel/originquality.h>
#include <seiscomp/datamodel/pick.h>
#include <seiscomp/datamodel/waveformstreamid.h>

#include <boost/filesystem.hpp>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>

namespace fs = boost::filesystem;

namespace Seiscomp {
namespace detect {
namespace test {

namespace cli {

class Flag;
std::string to_string(const Flag &flag);

class Flag {
 public:
  virtual std::string flag() const = 0;
  friend std::ostream &operator<<(std::ostream &os, const Flag &flag);

 protected:
  virtual void to_string(std::ostream &os) const;
};

class ArgFlag : public Flag {
 public:
  explicit ArgFlag(const std::string &arg);

 protected:
  void to_string(std::ostream &os) const override;

  void setArg(const std::string &arg);

 private:
  std::string _arg;
};

class BooleanFlag : public ArgFlag {
 public:
  BooleanFlag();
  explicit BooleanFlag(bool enabled);
  void enable();
  void disable();
};

class FlagDebug : public Flag {
 public:
  std::string flag() const override;
};

class FlagConsole : public BooleanFlag {
 public:
  std::string flag() const override;
};

class FlagOffline : public Flag {
 public:
  std::string flag() const override;
};

class FlagAmplitudesForce : public BooleanFlag {
 public:
  explicit FlagAmplitudesForce(bool enabled);
  std::string flag() const override;
};

class FlagMagnitudesForce : public BooleanFlag {
 public:
  explicit FlagMagnitudesForce(bool enabled);
  std::string flag() const override;
};

class FlagPlayback : public Flag {
 public:
  std::string flag() const override;
};

class FlagTemplatesReload : public Flag {
 public:
  std::string flag() const override;
};

class FlagAgencyId : public ArgFlag {
 public:
  explicit FlagAgencyId(const std::string &id);
  std::string flag() const override;
};

class FlagAuthor : public ArgFlag {
 public:
  explicit FlagAuthor(const std::string &name);
  std::string flag() const override;
};

class FlagPlugins : public ArgFlag {
 public:
  explicit FlagPlugins(const std::string &plugin);
  explicit FlagPlugins(const std::vector<std::string> &plugins);

  std::string flag() const override;
};

class FlagEp : public ArgFlag {
 public:
  explicit FlagEp(const std::string &fpath);
  explicit FlagEp(const fs::path &fpath);
  std::string flag() const override;
};

class FlagConfigFile : public ArgFlag {
 public:
  explicit FlagConfigFile(const std::string &fpath);
  explicit FlagConfigFile(const fs::path &fpath);
  std::string flag() const override;
};

class FlagDB : public ArgFlag {
 public:
  explicit FlagDB(const std::string &uri);
  explicit FlagDB(const fs::path &fpath);
  std::string flag() const override;
};

class FlagConfigModule : public ArgFlag {
 public:
  explicit FlagConfigModule(const std::string &configModule);
  std::string flag() const override;
};

class FlagInventoryDB : public ArgFlag {
 public:
  explicit FlagInventoryDB(const std::string &uri);
  explicit FlagInventoryDB(const fs::path &fpath);
  std::string flag() const override;
};

class FlagConfigDB : public ArgFlag {
 public:
  explicit FlagConfigDB(const std::string &uri);
  explicit FlagConfigDB(const fs::path &fpath);
  std::string flag() const override;
};

class FlagEventDB : public ArgFlag {
 public:
  explicit FlagEventDB(const std::string &uri);
  explicit FlagEventDB(const fs::path &fpath);
  std::string flag() const override;
};

class FlagRecordURL : public ArgFlag {
 public:
  explicit FlagRecordURL(const std::string &url);
  std::string flag() const override;
};

class FlagRecordStartTime : public ArgFlag {
 public:
  explicit FlagRecordStartTime(const std::string &timeStr);
  std::string flag() const override;
};

class FlagRecordEndTime : public ArgFlag {
 public:
  explicit FlagRecordEndTime(const std::string &timeStr);
  std::string flag() const override;
};

class FlagTemplatesJSON : public ArgFlag {
 public:
  explicit FlagTemplatesJSON(const std::string &fpath);
  explicit FlagTemplatesJSON(const fs::path &fpath);
  std::string flag() const override;
};

}  // namespace cli

/* -------------------------------------------------------------------------- */
// Compare `DataModel::EventParameters element-wise
void eventParametersCmp(const DataModel::EventParametersCPtr &lhs,
                        const DataModel::EventParametersCPtr &rhs);

// Compare `DataModel::Pick` element-wise
void pickCmp(const DataModel::PickCPtr &lhs, const DataModel::PickCPtr &rhs);

// Compare `DataModel::WaveformStreamID` element wise
void waveformStreamIdCmp(const DataModel::WaveformStreamID &lhs,
                         const DataModel::WaveformStreamID &rhs);

// Compare `DataModel::Origin` element-wise
void originCmp(const DataModel::OriginCPtr &lhs,
               const DataModel::OriginCPtr &rhs);

// Compare `DataModel::OriginQuality` element-wise
void originQualityCmp(const DataModel::OriginQualityCPtr &lhs,
                      const DataModel::OriginQualityCPtr &rhs);

// Compare `DataModel::Arrival` element-wise
void arrivalCmp(const DataModel::ArrivalCPtr &lhs,
                const DataModel::ArrivalCPtr &rhs);

// Compare `DataModel::Magnitude` element-wise
void magnitudeCmp(const DataModel::MagnitudeCPtr &lhs,
                  const DataModel::MagnitudeCPtr &rhs);

// Compare `DataModel::Amplitude` element-wise
void amplitudeCmp(const DataModel::AmplitudeCPtr &lhs,
                  const DataModel::AmplitudeCPtr &rhs);

// Compare `DataModel::Comment` element-wise
void commentCmp(const DataModel::CommentCPtr &lhs,
                const DataModel::CommentCPtr &rhs);

/* -------------------------------------------------------------------------- */
struct TempDirFixture {
  TempDirFixture();
  TempDirFixture(bool keepTempdir);
  ~TempDirFixture();

  std::string pathTempdirStr() const;
  const char *pathTempdirCStr() const;

  fs::path pathTempdir;

 protected:
  static fs::path createPathUnique();

  void createTempdir();

 private:
  static const std::string _pathSubDir;
  // Maximum number of tries in order to create the temporary directory
  static const int _maxTries;

  bool _keepTempdir{false};
};

/* -------------------------------------------------------------------------- */
template <typename TApp>
class ApplicationWrapper {
 public:
  ApplicationWrapper(const std::vector<std::string> &argv);
  ~ApplicationWrapper();

  int operator()();

 private:
  std::vector<char *> _argv;
};

}  // namespace test
}  // namespace detect
}  // namespace Seiscomp

#include "integration_utils.ipp"

#endif  // SCDETECT_APPS_SCDETECT_TEST_INTEGRATION_UTILS_H_
