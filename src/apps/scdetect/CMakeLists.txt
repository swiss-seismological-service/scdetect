set(DETECT_TARGET scdetect)

set(
  DETECT_SOURCES
    app.cpp
    config.cpp
    detector.cpp
    detector/arrival.cpp
    detector/detector.cpp
    detector/linker.cpp
    detector/pot.cpp
    eventstore.cpp
    exception.cpp
    main.cpp
    processor.cpp
    template.cpp
    utils.cpp
    validators.cpp
    waveform.cpp
    waveformprocessor.cpp
)

set(
  DETECT_HEADERS
    app.h
    builder.h
    config.h
    detector.h
    detector/arrival.h
    detector/detector.h
    detector/linker.h
    detector/pot.h
    eventstore.h
    exception.h
    log.h
    processor.h
    settings.h
    template.h
    utils.h
    validators.h
    version.h
    waveform.h
    waveformprocessor.h
)

# add_definitions(
#   -DSCDETECT_DEBUG
# )

if (${CMAKE_BUILD_TYPE} EQUAL "DEBUG")
  add_compile_options(-fsanitize=address)
  add_link_options(-fsanitize=address)
endif()


sc_add_executable(DETECT ${DETECT_TARGET})
sc_link_libraries_internal(${DETECT_TARGET} client)
sc_install_init(${DETECT_TARGET}
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../../../base/common/apps/templates/initd.py")

file(GLOB descs "${CMAKE_CURRENT_SOURCE_DIR}/descriptions/*.xml")
install(FILES ${descs} DESTINATION ${SC3_PACKAGE_APP_DESC_DIR})
