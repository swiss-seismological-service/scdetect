ADD_DEFINITIONS("-DTEST_BUILD_DIR=\"${CMAKE_CURRENT_BINARY_DIR}\"")

SET(TESTS
  template_xcorr.cpp
  utils_cma.cpp
)

SET(SOURCES_template_xcorr
  ../exception.cpp
  ../processor.cpp
  ../utils.cpp
  ../template.cpp
  ../waveform.cpp
)

SET(SOURCES_utils_cma
  ../exception.cpp
  ../utils.cpp
)

FOREACH(test_src ${TESTS})
	GET_FILENAME_COMPONENT(test_fname ${test_src} NAME_WE)
	SET(test_name test_scdetect_${test_fname})
  ADD_EXECUTABLE(${test_name} ${test_src} ${SOURCES_${test_fname}})
	SC_LINK_LIBRARIES_INTERNAL(${test_name} unittest core client)
	SC_LINK_LIBRARIES(${test_name} ${Boost_unit_test_framework_LIBRARY})

	ADD_TEST(
		NAME ${test_name}
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		COMMAND ${test_name}
	)
ENDFOREACH(test_src)
