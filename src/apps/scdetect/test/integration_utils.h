#ifndef SCDETECT_APPS_SCDETECT_TEST_INTEGRATION_UTILS_H_
#define SCDETECT_APPS_SCDETECT_TEST_INTEGRATION_UTILS_H_

#include <ostream>
#include <sstream>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>

#include <seiscomp/datamodel/arrival.h>
#include <seiscomp/datamodel/eventparameters.h>
#include <seiscomp/datamodel/magnitude.h>
#include <seiscomp/datamodel/origin.h>
#include <seiscomp/datamodel/originquality.h>
#include <seiscomp/datamodel/pick.h>

namespace fs = boost::filesystem;

namespace Seiscomp {
namespace detect {
namespace test {

namespace cli {

class Flag;
std::string to_string(const Flag &flag);

class Flag {
public:
  virtual const std::string flag() const = 0;
  friend std::ostream &operator<<(std::ostream &os, const Flag &flag);

protected:
  virtual void to_string(std::ostream &os) const;
};

class ArgFlag : public Flag {
public:
  ArgFlag(const std::string &arg);

protected:
  void to_string(std::ostream &os) const override;

private:
  std::string arg_;
};

class FlagDebug : public Flag {
public:
  const std::string flag() const override;
};

class FlagConsole : public ArgFlag {
public:
  FlagConsole();
  const std::string flag() const override;
};

class FlagOffline : public ArgFlag {
public:
  FlagOffline();
  const std::string flag() const override;
};

class FlagTemplatesReload : public ArgFlag {
public:
  FlagTemplatesReload();
  const std::string flag() const override;
};

class FlagAgencyId : public ArgFlag {
public:
  FlagAgencyId(const std::string &id);
  const std::string flag() const override;
};

class FlagAuthor : public ArgFlag {
public:
  FlagAuthor(const std::string &id);
  const std::string flag() const override;
};

class FlagPlugins : public ArgFlag {
public:
  FlagPlugins(const std::string &plugin);
  FlagPlugins(const std::vector<std::string> &plugins);

  const std::string flag() const override;
};

class FlagEp : public ArgFlag {
public:
  explicit FlagEp(const std::string &fpath);
  explicit FlagEp(const fs::path &fpath);
  const std::string flag() const override;
};

class FlagDB : public ArgFlag {
public:
  explicit FlagDB(const std::string &uri);
  explicit FlagDB(const fs::path &fpath);
  const std::string flag() const override;
};

class FlagInventoryDB : public ArgFlag {
public:
  explicit FlagInventoryDB(const std::string &uri);
  explicit FlagInventoryDB(const fs::path &fpath);
  const std::string flag() const override;
};

class FlagEventDB : public ArgFlag {
public:
  explicit FlagEventDB(const std::string &uri);
  explicit FlagEventDB(const fs::path &fpath);
  const std::string flag() const override;
};

class FlagRecordURL : public ArgFlag {
public:
  explicit FlagRecordURL(const std::string &url);
  const std::string flag() const override;
};

class FlagRecordStartTime : public ArgFlag {
public:
  FlagRecordStartTime(const std::string &time_str);
  const std::string flag() const override;
};

class FlagRecordEndTime : public ArgFlag {
public:
  FlagRecordEndTime(const std::string &time_str);
  const std::string flag() const override;
};

class FlagTemplatesJSON : public ArgFlag {
public:
  explicit FlagTemplatesJSON(const std::string &fpath);
  explicit FlagTemplatesJSON(const fs::path &fpath);
  const std::string flag() const override;
};

} // namespace cli

/* -------------------------------------------------------------------------- */
// Compare `DataModel::EventParameters element-wise
void EventParametersCmp(const DataModel::EventParametersCPtr &lhs,
                        const DataModel::EventParametersCPtr &rhs);

// Compare `DataModel::Pick` element-wise
void PickCmp(const DataModel::PickCPtr &lhs, const DataModel::PickCPtr &rhs);

// Compare `DataModel::Origin` element-wise
void OriginCmp(const DataModel::OriginCPtr &lhs,
               const DataModel::OriginCPtr &rhs);

// Compare `DataModel::OriginQuality` element-wise
void OriginQualityCmp(const DataModel::OriginQualityCPtr &lhs,
                      const DataModel::OriginQualityCPtr &rhs);

// Compare `DataModel::Arrival` element-wise
void ArrivalCmp(const DataModel::ArrivalCPtr &lhs,
                const DataModel::ArrivalCPtr &rhs);

// Compare `DataModel::Magnitude` element-wise
void MagnitudeCmp(const DataModel::MagnitudeCPtr &lhs,
                  const DataModel::MagnitudeCPtr &rhs);

/* -------------------------------------------------------------------------- */
struct TempDirFixture {
  TempDirFixture();
  TempDirFixture(bool keep_tempdir);
  ~TempDirFixture();

  const std::string path_tempdir_str() const;
  const char *path_tempdir_cstr() const;

  fs::path path_tempdir;

protected:
  static const fs::path CreatePathUnique();

  void CreateTempdir();

private:
  static const std::string path_subdir;
  // Maximum number of tries in order to create the temporary directory
  static const int max_tries;

  bool keep_tempdir_{false};
};

/* -------------------------------------------------------------------------- */
struct CLIParserFixture {
  CLIParserFixture();
  ~CLIParserFixture();

  void setup();
  void teardown();

  static fs::path path_data;
  static bool keep_tempdir;
};

/* -------------------------------------------------------------------------- */
template <typename TApp> class ApplicationWrapper {
public:
  ApplicationWrapper(const std::vector<std::string> &argv);
  ~ApplicationWrapper();

  int operator()();

private:
  std::vector<char *> argv_;
};

#include "integration_utils.ipp"

} // namespace test
} // namespace detect
} // namespace Seiscomp

#endif // SCDETECT_APPS_SCDETECT_TEST_INTEGRATION_UTILS_H_
