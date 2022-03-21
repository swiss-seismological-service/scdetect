set(BENCHMARKS
  app.cpp
)

set(UTILS
  prepare_waveform_data.cpp
)

set(SOURCES_app
  ../amplitude/factory.cpp
  ../amplitude/ratio.cpp
  ../amplitude/mlx.cpp
  ../amplitude/mrelative.cpp
  ../amplitude/rms.cpp
  ../amplitude_processor.cpp
  ../combining_amplitude_processor.cpp
  ../reducing_amplitude_processor.cpp
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
  ../operator/buffer.cpp
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
)

set(SOURCES_prepare_waveform_data
  ../config/detector.cpp
  ../config/validators.cpp
  ../exception.cpp
  ../log.cpp
  ../util/util.cpp
  ../util/waveform_stream_id.cpp
  ../resamplerstore.cpp
  ../waveform.cpp
)
  

find_package(SQLite3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS timer program_options)
set(THREADPOOL_INCLUDE_DIR
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../libs/external/thread-pool)

foreach(BENCHMARK_SRC ${BENCHMARKS})
  get_filename_component(BENCHMARK ${BENCHMARK_SRC} NAME_WE)
  set(PERF_TARGET perf_scdetect_cc_${BENCHMARK})
  add_executable(${PERF_TARGET} ${BENCHMARK_SRC} ${SOURCES_${BENCHMARK}})
  target_link_libraries(${PERF_TARGET} ${SQLITE3_LIBRARIES} ${Boost_LIBRARIES})
  target_include_directories(${PERF_TARGET} PRIVATE ${THREADPOOL_INCLUDE_DIR})
  sc_link_libraries_internal(${PERF_TARGET} core client)
endforeach()

foreach(UTIL_SRC ${UTILS})
  get_filename_component(UTIL ${UTIL_SRC} NAME_WE)
  set(UTIL_TARGET perf_util_scdetect_cc_${UTIL})
  add_executable(${UTIL_TARGET} ${UTIL_SRC} ${SOURCES_${UTIL}})
  target_link_libraries(${UTIL_TARGET} ${Boost_LIBRARIES})
  target_include_directories(${UTIL_TARGET} PRIVATE ${THREADPOOL_INCLUDE_DIR})
  sc_link_libraries_internal(${UTIL_TARGET} core)
endforeach()

