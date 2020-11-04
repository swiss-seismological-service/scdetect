SET(DETECT_TARGET scdetect)

SET(
  DETECT_SOURCES
    app.cpp
    config.cpp
    detector.cpp
    main.cpp
    processor.cpp
    template.cpp
    utils.cpp
    waveform.cpp
    xcorr.cpp
)

SET(
  DETECT_HEADERS
    app.h
    config.h
    detector.h
    settings.h
    processor.cpp
    template.h
    utils.h
    version.h
    waveform.h
    xcorr.h
)

SC_ADD_EXECUTABLE(DETECT ${DETECT_TARGET})
SC_LINK_LIBRARIES_INTERNAL(${DETECT_TARGET} client)
SC_INSTALL_INIT(${DETECT_TARGET} ${CMAKE_CURRENT_SOURCE_DIR}/../../../../../base/common/apps/templates/initd.py)

FILE(GLOB descs "${CMAKE_CURRENT_SOURCE_DIR}/descriptions/*.xml")
INSTALL(FILES ${descs} DESTINATION ${SC3_PACKAGE_APP_DESC_DIR})
