set(TESTS
  filter_crosscorrelation.cpp
  integration.cpp
  reducingamplitudeprocessor.cpp
  utils_cma.cpp
)

SET(SOURCES_filter_crosscorrelation
  ../exception.cpp
  ../filter.cpp
  ../filter/crosscorrelation.h
  ../resamplerstore.cpp
  ../utils.cpp
  ../waveform.cpp
)

set(SOURCES_utils_cma
  ../exception.cpp
  ../utils.cpp
)

set(SOURCES_integration
  ../amplitude/rms.cpp
  ../amplitudeprocessor.cpp
  ../app.cpp
  ../binding.cpp
  ../builder.cpp
  ../config.cpp
  ../datamodel/ddl.cpp
  ../detail/sqlite.cpp
  ../detector/arrival.cpp
  ../detector/detector.cpp
  ../detector/detectorbuilder.cpp
  ../detector/detectorwaveformprocessor.cpp
  ../detector/linker/association.cpp
  ../detector/linker/strategy.cpp
  ../detector/linker.cpp
  ../detector/pot.cpp
  ../detector/templatewaveformprocessor.cpp
  ../eventstore.cpp
  ../exception.cpp
  ../filter.cpp
  ../log.cpp
  ../magnitudeprocessor.cpp
  ../magnitude/regression.cpp
  ../operator/resample.cpp
  ../operator/ringbuffer.cpp
  ../processor.cpp
  ../resamplerstore.cpp
  ../stream.cpp
  ../templatefamily.cpp
  ../timewindowprocessor.cpp
  ../utils.cpp
  ../validators.cpp
  ../waveform.cpp
  ../waveformoperator.cpp
  ../waveformprocessor.cpp
  fixture.cpp
  integration_utils.cpp
)

set(SOURCES_reducingamplitudeprocessor
  ../amplitudeprocessor.cpp
  ../exception.cpp
  ../processor.cpp
  ../stream.cpp
  ../timewindowprocessor.cpp
  ../resamplerstore.cpp
  ../utils.cpp
  ../waveform.cpp
  ../waveformoperator.cpp
  ../waveformprocessor.cpp
  fixture.cpp
)


add_definitions("-DTEST_BUILD_DIR=\"${CMAKE_CURRENT_BINARY_DIR}\"")
# add_compile_options(-fsanitize=address -fno-omit-frame-pointer)
# add_link_options(-fsanitize=address)

foreach(TEST_SRC ${TESTS})
	get_filename_component(TEST_FNAME ${TEST_SRC} NAME_WE)
	set(TEST_TARGET test_scdetect_${TEST_FNAME})
  add_executable(${TEST_TARGET} ${TEST_SRC} ${SOURCES_${TEST_FNAME}})
	sc_link_libraries_internal(${TEST_TARGET} unittest core client)
	sc_link_libraries(${TEST_TARGET} ${Boost_unit_test_framework_LIBRARY})
  target_link_libraries(${TEST_TARGET} ${SQLITE3_LIBRARIES})

	add_test(
		NAME ${TEST_TARGET}
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		COMMAND ${TEST_TARGET} --log_level=message -- "${CMAKE_CURRENT_SOURCE_DIR}/data"
	)
endforeach(TEST_SRC)
