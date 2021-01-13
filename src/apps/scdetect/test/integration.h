#ifndef SCDETECT_APPS_SCDETECT_TEST_INTEGRATION_H_
#define SCDETECT_APPS_SCDETECT_TEST_INTEGRATION_H_

#include <ostream>
#include <sstream>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>

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

const std::string PathData(const std::string &fname = "");

std::vector<char *>
StringsToCStrings(const std::vector<std::string> &v_strings);

/* -------------------------------------------------------------------------- */
struct TempDirFixture {
  TempDirFixture();
  ~TempDirFixture();

  const std::string path_tempdir_str() const;
  const char *path_tempdir_cstr() const;

  static const fs::path create_path_unique();
  fs::path path_tempdir;

private:
  static const std::string path_subdir;
};

} // namespace test
} // namespace detect
} // namespace Seiscomp

#endif // SCDETECT_APPS_SCDETECT_TEST_INTEGRATION_H_
