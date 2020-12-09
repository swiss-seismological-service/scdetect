ADD_DEFINITIONS("-DTEST_BUILD_DIR=\"${CMAKE_CURRENT_BINARY_DIR}\"")

SET(TESTS
  template_xcorr.cpp
)

LIST(REMOVE_ITEM DETECT_SOURCES "main.cpp")
LIST(TRANSFORM DETECT_SOURCES PREPEND "../")

FOREACH(test_src ${TESTS})
	GET_FILENAME_COMPONENT(test_fname ${test_src} NAME_WE)
	SET(test_name test_scdetect_${test_fname})
  ADD_EXECUTABLE(${test_name} ${test_src} ${DETECT_SOURCES})
	SC_LINK_LIBRARIES_INTERNAL(${test_name} unittest core client)
	SC_LINK_LIBRARIES(${test_name} ${Boost_unit_test_framework_LIBRARY})

	ADD_TEST(
		NAME ${test_name}
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		COMMAND ${test_name}
	)
ENDFOREACH(test_src)
