#include "utils.h"

#include <seiscomp/core/strings.h>

#include "settings.h"

namespace Seiscomp {
namespace detect {
namespace utils {

bool ValidatePhase(const std::string &phase) {
  return phase == "auto" ||
         find(settings::kValidPhases.begin(), settings::kValidPhases.end(),
              phase) != settings::kValidPhases.end();
}

bool ValidateXCorrThreshold(const double &thres) {
  return 0 <= thres && 1 >= thres;
}

bool ValidateStreamID(const std::string &id) {
  std::vector<std::string> tokens;
  return 4 == Core::split(tokens, id, ".", false);
}

} // namespace utils
} // namespace detect
} // namespace Seiscomp
