#include "log.h"

#include <sstream>

namespace Seiscomp {
namespace detect {
namespace logging {

const char TaggedMessage::_tagBegin = '[';
const char TaggedMessage::_tagEnd = ']';

TaggedMessage::TaggedMessage(const std::string& tag, const std::string& text)
    : _tag{tag}, _text{text} {}

void TaggedMessage::setText(const std::string& text) { _text = text; }

std::ostream& operator<<(std::ostream& os, const TaggedMessage& m) {
  return os << m._tagBegin << m._tag << m._tagEnd << " " << m._text;
}

std::string to_string(const TaggedMessage& m) {
  std::ostringstream oss;
  oss << m;
  return oss.str();
}

}  // namespace logging
}  // namespace detect
}  // namespace Seiscomp
