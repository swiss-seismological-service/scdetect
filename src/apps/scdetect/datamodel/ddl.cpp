#include "ddl.h"

#include <algorithm>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include <boost/algorithm/string/join.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/filesystem.hpp>

#include <seiscomp/core/exceptions.h>
#include <seiscomp/system/environment.h>

namespace Seiscomp {
namespace DataModel {

void createAll(IO::DatabaseInterface *dbDriver) {
  if (!dbDriver) {
    return;
  }

  boost::filesystem::path pathDDL{"db"};
  const std::string className{dbDriver->className()};
  if ("sqlite3_database_interface_" == className ||
      "sqlite3_database_interface" == className) {
    pathDDL /= "sqlite3.sql";
  } else if ("postgresql_database_interface" == className) {
    pathDDL /= "postgresql.sql";
  } else if ("mysql_database_interface" == className) {
    pathDDL /= "mysql.sql";
  } else {
    throw Core::ValueException("Unknown DB driver class name: " + className);
  }

  std::vector<std::string> tokens;
  int blockCounter{0};
  auto processDDL = [&tokens, &blockCounter,
                     &dbDriver](const std::string &line) {
    tokens.push_back(line);

    // check for blocks
    std::vector<std::string> tmp;
    boost::algorithm::find_all(tmp, line, "BEGIN");
    blockCounter += tmp.size();
    tmp.clear();
    boost::algorithm::find_all(tmp, line, "END;");
    blockCounter -= tmp.size();

    if (blockCounter || line.back() != ';') {
      return;
    }

    dbDriver->start();
    dbDriver->execute(boost::algorithm::join(tokens, " ").c_str());
    dbDriver->commit();
    tokens.clear();
  };

  // read SQL DDL file
  Environment *env{Environment::Instance()};
  pathDDL = env->shareDir() / pathDDL;

  std::ifstream ifs;
  ifs.exceptions(std::ifstream::badbit); // No need to check failbit
  try {
    ifs.open(pathDDL.string());
    std::string line;
    while (std::getline(ifs, line)) {
      boost::algorithm::trim(line);
      if (!line.empty()) {
        processDDL(line);
      }
    }
  } catch (const std::ifstream::failure &e) {
    throw Core::GeneralException{std::string{"Failed to open/read DDL file: "} +
                                 e.what()};
  }

  ifs.close();
}

} // namespace DataModel
} // namespace Seiscomp
