set(DETECT_TARGET scdetect)

set(
  DETECT_SOURCES
    amplitude/rms.cpp
    amplitudeprocessor.cpp
    app.cpp
    binding.cpp
    config/detector.cpp
    config/exception.cpp
    config/template_family.cpp
    config/validators.cpp
    datamodel/ddl.cpp
    detail/sqlite.cpp
    detector/arrival.cpp
    detector/detector.cpp
    detector/detectorbuilder.cpp
    detector/detectorwaveformprocessor.cpp
    detector/linker/association.cpp
    detector/linker/pot.cpp
    detector/linker/strategy.cpp
    detector/linker.cpp
    detector/templatewaveformprocessor.cpp
    eventstore.cpp
    exception.cpp
    filter.cpp
    log.cpp
    magnitudeprocessor.cpp
    magnitude/decorator/range.cpp
    magnitude/decorator.cpp
    magnitude/regression.cpp
    magnitude/util.cpp
    magnitude/templatefamily.cpp
    main.cpp
    operator/resample.cpp
    operator/ringbuffer.cpp
    processor.cpp
    resamplerstore.cpp
    stream.cpp
    templatefamily.cpp
    timewindowprocessor.cpp
    util/horizontal_components.cpp
    util/util.cpp
    util/waveform_stream_id.cpp
    waveform.cpp
    waveformoperator.cpp
    waveformprocessor.cpp
)

# add_definitions(
#   -DSCDETECT_DEBUG
# )

if ("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
  add_compile_options(-fsanitize=address)
  add_link_options(-fsanitize=address)
endif()


sc_add_executable(DETECT ${DETECT_TARGET})
find_package(SQLite3 REQUIRED)
target_link_libraries(${DETECT_TARGET} ${SQLITE3_LIBRARIES})
sc_link_libraries_internal(${DETECT_TARGET} config client)
sc_install_init(${DETECT_TARGET}
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../../../base/common/apps/templates/initd.py")

file(GLOB descs "${CMAKE_CURRENT_SOURCE_DIR}/descriptions/*.xml")
install(FILES ${descs} DESTINATION ${SC3_PACKAGE_APP_DESC_DIR})
