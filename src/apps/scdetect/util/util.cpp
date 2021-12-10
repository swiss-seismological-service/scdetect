#include "util.h"

#include <seiscomp/utils/files.h>

#include <boost/algorithm/string.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

namespace Seiscomp {
namespace detect {
namespace util {

const std::string createUUID() {
  auto uuid{boost::uuids::random_generator{}()};
  return boost::uuids::to_string(uuid);
}

void replaceEscapedXMLFilterIdChars(std::string &str) {
  boost::replace_all(str, "&gt;", ">");
}

bool createDirectory(const boost::filesystem::path &p) {
  if (!Util::pathExists(p.string()) && !Util::createPath(p.string())) {
    return false;
  }
  return true;
}

}  // namespace util
}  // namespace detect
}  // namespace Seiscomp
