set(UNIT_TESTS
  filter_crosscorrelation.cpp
  util_math_cma.cpp
)

set(INTEGRATION_TESTS
  integration.cpp
)

SET(SOURCES_filter_crosscorrelation
  ../exception.cpp
  ../filter.cpp
  ../resamplerstore.cpp
  ../template_waveform.cpp
  ../util/filter.cpp
  ../util/util.cpp
  ../util/waveform_stream_id.cpp
  ../waveform.cpp
)

set(SOURCES_util_math_cma
  ../exception.cpp
)

set(SOURCES_integration
  ../amplitude/factory.cpp
  ../amplitude/ratio.cpp
  ../amplitude/mlx.cpp
  ../amplitude/mrelative.cpp
  ../amplitude/rms.cpp
  ../amplitude/util.cpp
  ../amplitude_processor.cpp
  ../combining_amplitude_processor.cpp
  ../app.cpp
  ../binding.cpp
  ../builder.cpp
  ../config/detector.cpp
  ../config/exception.cpp
  ../config/template_family.cpp
  ../config/validators.cpp
  ../datamodel/ddl.cpp
  ../detail/sqlite.cpp
  ../detector/arrival.cpp
  ../detector/detector.cpp
  ../detector/detector_impl.cpp
  ../detector/linker/association.cpp
  ../detector/linker/pot.cpp
  ../detector/linker.cpp
  ../detector/template_waveform_processor.cpp
  ../eventstore.cpp
  ../exception.cpp
  ../filter.cpp
  ../log.cpp
  ../magnitude_processor.cpp
  ../magnitude/decorator/range.cpp
  ../magnitude/decorator.cpp
  ../magnitude/factory.cpp
  ../magnitude/mlx.cpp
  ../magnitude/mrelative.cpp
  ../magnitude/util.cpp
  ../magnitude/template_family.cpp
  ../operator/resample.cpp
  ../operator/ringbuffer.cpp
  ../processing/detail/gap_interpolate.cpp
  ../processing/processor.cpp
  ../processing/stream.cpp
  ../processing/timewindow_processor.cpp
  ../processing/waveform_operator.cpp
  ../processing/waveform_processor.cpp
  ../resamplerstore.cpp
  ../template_family.cpp
  ../template_waveform.cpp
  ../util/filter.cpp
  ../util/horizontal_components.cpp
  ../util/util.cpp
  ../util/waveform_stream_id.cpp
  ../waveform.cpp
  fixture.cpp
  integration_utils.cpp
)

add_definitions("-DTEST_BUILD_DIR=\"${CMAKE_CURRENT_BINARY_DIR}\"")

foreach(TEST_SRC ${UNIT_TESTS})
  get_filename_component(TEST_FNAME ${TEST_SRC} NAME_WE)
  set(TEST_TARGET test_scdetect_cc_${TEST_FNAME})
  add_executable(${TEST_TARGET} ${TEST_SRC} ${SOURCES_${TEST_FNAME}})
  sc_link_libraries_internal(${TEST_TARGET} unittest core client)
  sc_link_libraries(${TEST_TARGET} ${Boost_unit_test_framework_LIBRARY})
  target_link_libraries(${TEST_TARGET} ${SQLITE3_LIBRARIES})

  add_test(
    NAME ${TEST_TARGET}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMAND ${TEST_TARGET} --log_level=message
  )
endforeach()

find_package(SQLite3 REQUIRED)
foreach(TEST_SRC ${INTEGRATION_TESTS})
  get_filename_component(TEST_FNAME ${TEST_SRC} NAME_WE)
  set(TEST_TARGET test_scdetect_cc_${TEST_FNAME})
  add_executable(${TEST_TARGET} ${TEST_SRC} ${SOURCES_${TEST_FNAME}})
  target_link_libraries(${TEST_TARGET} ${SQLITE3_LIBRARIES})
  sc_link_libraries_internal(${TEST_TARGET} unittest core client)
  sc_link_libraries(${TEST_TARGET} ${Boost_unit_test_framework_LIBRARY})
  target_link_libraries(${TEST_TARGET} ${SQLITE3_LIBRARIES})

  add_test(
    NAME ${TEST_TARGET}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMAND ${TEST_TARGET} --log_level=message -- --path-data "${CMAKE_CURRENT_SOURCE_DIR}/data/integration/" "${CMAKE_CURRENT_SOURCE_DIR}/data/integration/dataset.conf"
  )
endforeach()

