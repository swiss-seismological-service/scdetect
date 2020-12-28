#ifndef SCDETECT_APPS_SCDETECT_LOG_H_
#define SCDETECT_APPS_SCDETECT_LOG_H_

#include <seiscomp/core/strings.h>
#define SEISCOMP_COMPONENT DETECT
#include <seiscomp/logging/log.h>

#define SCDETECT_LOG_DEBUG SEISCOMP_DEBUG
#define SCDETECT_LOG_INFO SEISCOMP_INFO
#define SCDETECT_LOG_WARNING SEISCOMP_WARNING
#define SCDETECT_LOG_ERROR SEISCOMP_ERROR
#define SCDETECT_LOG_NOTICE SEISCOMP_ERROR
#define SCDETECT_LOG SEISCOMP_LOG

#define SCDETECT_LOG_DEBUG_TAGGED(tag_str, ...)                                \
  SCDETECT_LOG_DEBUG((Core::stringify("[%s] ", tag_str.c_str()) +              \
                      Core::stringify(__VA_ARGS__))                            \
                         .c_str())
#define SCDETECT_LOG_INFO_TAGGED(tag_str, ...)                                 \
  SCDETECT_LOG_INFO((Core::stringify("[%s] ", tag_str.c_str()) +               \
                     Core::stringify(__VA_ARGS__))                             \
                        .c_str())
#define SCDETECT_LOG_WARNING_TAGGED(tag_str, ...)                              \
  SCDETECT_LOG_WARNING((Core::stringify("[%s] ", tag_str.c_str()) +            \
                        Core::stringify(__VA_ARGS__))                          \
                           .c_str())
#define SCDETECT_LOG_ERROR_TAGGED(tag_str, ...)                                \
  SCDETECT_LOG_ERROR((Core::stringify("[%s] ", tag_str.c_str()) +              \
                      Core::stringify(__VA_ARGS__))                            \
                         .c_str())
#define SCDETECT_LOG_NOTICE_TAGGED(tag_str, format, ...)                       \
  SCDETECT_LOG_NOTICE((Core::stringify("[%s] ", tag_str.c_str()) +             \
                       Core::stringify(__VA_ARGS__))                           \
                          .c_str())
#define SCDETECT_LOG_TAGGED(channel, tag_str, ...)                             \
  SCDETECT_LOG(channel, (Core::stringify("[%s] ", tag_str.c_str()) +           \
                         Core::stringify(__VA_ARGS__))                         \
                            .c_str())

#endif // SCDETECT_APPS_SCDETECT_LOG_H_
