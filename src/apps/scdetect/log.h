#ifndef SCDETECT_APPS_SCDETECT_LOG_H_
#define SCDETECT_APPS_SCDETECT_LOG_H_

#define SEISCOMP_COMPONENT DETECT
#include <seiscomp/logging/log.h>

#define SCDETECT_DEBUG_TAGGED(tag_str, format, ...)                            \
  SEISCOMP_DEBUG("[%s] " format, tag_str.c_str(), ##__VA_ARGS__)
#define SCDETECT_INFO_TAGGED(tag_str, format, ...)                             \
  SEISCOMP_INFO("[%s] " format, tag_str.c_str(), ##__VA_ARGS__)
#define SCDETECT_WARNING_TAGGED(tag_str, format, ...)                          \
  SEISCOMP_WARNING("[%s] " format, tag_str.c_str(), ##__VA_ARGS__)
#define SCDETECT_ERROR_TAGGED(tag_str, format, ...)                            \
  SEISCOMP_WARNING("[%s] " format, tag_str.c_str(), ##__VA_ARGS__)
#define SCDETECT_NOTICE_TAGGED(tag_str, format, ...)                           \
  SEISCOMP_NOTICE("[%s] " format, tag_str.c_str(), ##__VA_ARGS__)
#define SCDETECT_LOG_TAGGED(channel, tag_str, format, ...)                     \
  SEISCOMP_LOG(channel, "[%s] " format, tag_str.c_str(), ##__VA_ARGS__)

#endif // SCDETECT_APPS_SCDETECT_LOG_H_
