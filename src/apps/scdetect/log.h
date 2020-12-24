#ifndef SCDETECT_APPS_SCDETECT_LOG_H_
#define SCDETECT_APPS_SCDETECT_LOG_H_

#define SEISCOMP_COMPONENT DETECT
#include <seiscomp/logging/log.h>

#define SCDETECT_LOG_DEBUG SEISCOMP_DEBUG
#define SCDETECT_LOG_INFO SEISCOMP_INFO
#define SCDETECT_LOG_WARNING SEISCOMP_WARNING
#define SCDETECT_LOG_ERROR SEISCOMP_ERROR
#define SCDETECT_LOG_NOTICE SEISCOMP_ERROR
#define SCDETECT_LOG SEISCOMP_LOG

#define SCDETECT_DEBUG_TAGGED(tag_str, format, ...)                            \
  SCDETECT_LOG_DEBUG("[%s] " format, tag_str.c_str(), ##__VA_ARGS__)
#define SCDETECT_INFO_TAGGED(tag_str, format, ...)                             \
  SCDETECT_LOG_INFO("[%s] " format, tag_str.c_str(), ##__VA_ARGS__)
#define SCDETECT_WARNING_TAGGED(tag_str, format, ...)                          \
  SCDETECT_LOG_WARNING("[%s] " format, tag_str.c_str(), ##__VA_ARGS__)
#define SCDETECT_ERROR_TAGGED(tag_str, format, ...)                            \
  SCDETECT_LOG_ERROR("[%s] " format, tag_str.c_str(), ##__VA_ARGS__)
#define SCDETECT_NOTICE_TAGGED(tag_str, format, ...)                           \
  SCDETECT_LOG_NOTICE("[%s] " format, tag_str.c_str(), ##__VA_ARGS__)
#define SCDETECT_LOG_TAGGED(channel, tag_str, format, ...)                     \
  SCDETECT_LOG(channel, "[%s] " format, tag_str.c_str(), ##__VA_ARGS__)

#endif // SCDETECT_APPS_SCDETECT_LOG_H_
