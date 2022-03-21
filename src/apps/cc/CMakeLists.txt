set(DETECT_TARGET scdetect-cc)

set(
  DETECT_SOURCES
    amplitude/factory.cpp
    amplitude/ratio.cpp
    amplitude/mlx.cpp
    amplitude/mrelative.cpp
    amplitude/rms.cpp
    amplitude_processor.cpp
    combining_amplitude_processor.cpp
    reducing_amplitude_processor.cpp
    app.cpp
    binding.cpp
    config/detector.cpp
    config/exception.cpp
    config/template_family.cpp
    config/validators.cpp
    datamodel/ddl.cpp
    detail/sqlite.cpp
    detector/arrival.cpp
    detector/detector_impl.cpp
    detector/detector.cpp
    detector/linker/association.cpp
    detector/linker/pot.cpp
    detector/linker.cpp
    detector/template_waveform_processor.cpp
    eventstore.cpp
    exception.cpp
    filter.cpp
    log.cpp
    magnitude_processor.cpp
    magnitude/decorator/range.cpp
    magnitude/decorator.cpp
    magnitude/factory.cpp
    magnitude/mlx.cpp
    magnitude/mrelative.cpp
    magnitude/util.cpp
    magnitude/template_family.cpp
    main.cpp
    operator/buffer.cpp
    operator/resample.cpp
    operator/ringbuffer.cpp
    processing/detail/gap_interpolate.cpp
    processing/processor.cpp
    processing/stream.cpp
    processing/timewindow_processor.cpp
    processing/waveform_operator.cpp
    processing/waveform_processor.cpp
    resamplerstore.cpp
    template_waveform.cpp
    template_family.cpp
    util/filter.cpp
    util/horizontal_components.cpp
    util/util.cpp
    util/waveform_stream_id.cpp
    waveform.cpp
)

if ("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
  add_compile_options(-fsanitize=address)
  add_link_options(-fsanitize=address)
endif()


find_package(SQLite3 REQUIRED)

set(THREADPOOL_INCLUDE_DIR
  ${CMAKE_CURRENT_SOURCE_DIR}/../../libs/external/thread-pool)

sc_add_executable(DETECT ${DETECT_TARGET})
target_compile_features(${DETECT_TARGET} PRIVATE cxx_std_17)
target_link_libraries(${DETECT_TARGET} ${SQLITE3_LIBRARIES})
target_include_directories(${DETECT_TARGET} PRIVATE ${THREADPOOL_INCLUDE_DIR})
sc_link_libraries_internal(${DETECT_TARGET} config client)
sc_install_init(${DETECT_TARGET}
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../../../base/common/apps/templates/initd.py")

file(GLOB descs "${CMAKE_CURRENT_SOURCE_DIR}/descriptions/*.xml")
install(FILES ${descs} DESTINATION ${SC3_PACKAGE_APP_DESC_DIR})

option("SCDETECT_CC_BUILD_BENCHMARKS" "Build the scdetect-cc benchmarks" OFF)
if (${SCDETECT_CC_BUILD_BENCHMARKS})
  add_subdirectory(perf)
endif()
