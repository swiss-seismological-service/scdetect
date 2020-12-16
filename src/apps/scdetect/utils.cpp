#include "utils.h"

#include <vector>

#include <seiscomp/core/strings.h>

#include "exception.h"
#include "settings.h"

namespace Seiscomp {
namespace detect {
namespace utils {

bool ValidateXCorrThreshold(const double &thres) {
  return 0 <= thres && 1 >= thres;
}

/* ------------------------------------------------------------------------- */
WaveformStreamID::WaveformStreamID(const std::string &net_sta_loc_cha) {
  std::vector<std::string> tokens;
  Core::split(tokens, net_sta_loc_cha, delimiter_.c_str(), false);

  if (4 != tokens.size()) {
    throw ValueException{std::string{"Invalid number of tokens: "} +
                         std::to_string(tokens.size())};
  }
  net_code_ = tokens[0];
  sta_code_ = tokens[1];
  loc_code_ = tokens[2];
  cha_code_ = tokens[3];
}

WaveformStreamID::WaveformStreamID(const std::string &net_code,
                                   const std::string &sta_code,
                                   const std::string &loc_code,
                                   const std::string &cha_code)
    : net_code_{net_code}, sta_code_{sta_code}, loc_code_{loc_code},
      cha_code_{cha_code} {}

const std::string &WaveformStreamID::net_code() const { return net_code_; }
const std::string &WaveformStreamID::sta_code() const { return sta_code_; }
const std::string &WaveformStreamID::loc_code() const { return loc_code_; }
const std::string &WaveformStreamID::cha_code() const { return cha_code_; }

bool WaveformStreamID::IsValid() const {
  return !(net_code_.empty() || sta_code_.empty() || cha_code_.empty());
}

std::ostream &operator<<(std::ostream &os, const WaveformStreamID &id) {
  os << id.net_code_ << id.delimiter_ << id.sta_code_ << id.delimiter_
     << id.loc_code_ << id.delimiter_ << id.cha_code_;
  return os;
}

} // namespace utils
} // namespace detect
} // namespace Seiscomp
