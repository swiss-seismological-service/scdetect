#ifndef SCDETECT_APPS_SCDETECT_LOG_H_
#define SCDETECT_APPS_SCDETECT_LOG_H_

#include <ostream>
#include <string>

#ifndef SEISCOMP_COMPONENT
#define SEISCOMP_COMPONENT DETECT
#endif

#include <seiscomp/core/strings.h>
#include <seiscomp/logging/log.h>

// XXX(damb): Avoid unused macro overriding method definition from
// <boost/property_tree/json_parser.hpp>
#undef expect

#define SCDETECT_LOG_DEBUG SEISCOMP_DEBUG
#define SCDETECT_LOG_INFO SEISCOMP_INFO
#define SCDETECT_LOG_WARNING SEISCOMP_WARNING
#define SCDETECT_LOG_ERROR SEISCOMP_ERROR
#define SCDETECT_LOG_NOTICE SEISCOMP_ERROR
#define SCDETECT_LOG SEISCOMP_LOG

#define SCDETECT_LOG_DEBUG_TAGGED(tag_str, ...)  \
  SCDETECT_LOG_DEBUG("[%s] %s", tag_str.c_str(), \
                     Core::stringify(__VA_ARGS__).c_str())
#define SCDETECT_LOG_INFO_TAGGED(tag_str, ...)  \
  SCDETECT_LOG_INFO("[%s] %s", tag_str.c_str(), \
                    Core::stringify(__VA_ARGS__).c_str())
#define SCDETECT_LOG_WARNING_TAGGED(tag_str, ...)  \
  SCDETECT_LOG_WARNING("[%s] %s", tag_str.c_str(), \
                       Core::stringify(__VA_ARGS__).c_str())
#define SCDETECT_LOG_ERROR_TAGGED(tag_str, ...)  \
  SCDETECT_LOG_ERROR("[%s] %s", tag_str.c_str(), \
                     Core::stringify(__VA_ARGS__).c_str())
#define SCDETECT_LOG_NOTICE_TAGGED(tag_str, format, ...) \
  SCDETECT_LOG_NOTICE("[%s] %s", tag_str.c_str(),        \
                      Core::stringify(__VA_ARGS__).c_str())
#define SCDETECT_LOG_TAGGED(channel, tag_str, ...)  \
  SCDETECT_LOG(channel, "[%s] %s", tag_str.c_str(), \
               Core::stringify(__VA_ARGS__).c_str())

namespace Seiscomp {
namespace detect {
namespace logging {

// Implements prefixing a message with a tag
class TaggedMessage {
 public:
  TaggedMessage(const std::string& tag, const std::string& text = "");
  // Sets the message text
  void setText(const std::string& text);

  friend std::ostream& operator<<(std::ostream& os, const TaggedMessage& m);

 private:
  static const char _tagBegin;
  static const char _tagEnd;
  std::string _tag;
  std::string _text;
};

std::string to_string(const TaggedMessage& m);

}  // namespace logging
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_LOG_H_
