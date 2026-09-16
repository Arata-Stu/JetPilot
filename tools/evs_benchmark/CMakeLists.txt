cmake_minimum_required(VERSION 3.18)
project(evs_benchmark LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(EVS_BENCH_ENABLE_CUDA "Build the native CUDA backend when a CUDA compiler is available" ON)
option(EVS_BENCH_ENABLE_METAVISION "Build the native RAW-to-EVSBIN converter" ON)

add_library(evs_benchmark_core
  src/dataset.cpp
  src/cpu_backend.cpp
)
target_include_directories(evs_benchmark_core PUBLIC include)
target_compile_options(evs_benchmark_core PRIVATE -Wall -Wextra -Wpedantic)

include(CheckLanguage)
if(EVS_BENCH_ENABLE_CUDA)
  check_language(CUDA)
endif()

if(EVS_BENCH_ENABLE_CUDA AND CMAKE_CUDA_COMPILER)
  enable_language(CUDA)
  add_library(evs_benchmark_gpu src/cuda_backend.cu)
  target_include_directories(evs_benchmark_gpu PUBLIC include)
  target_compile_definitions(evs_benchmark_gpu PUBLIC EVS_BENCH_HAS_CUDA=1)
  set_target_properties(evs_benchmark_gpu PROPERTIES
    CUDA_STANDARD 17
    CUDA_STANDARD_REQUIRED ON
    CUDA_SEPARABLE_COMPILATION ON
  )
  target_compile_options(evs_benchmark_gpu PRIVATE
    $<$<COMPILE_LANGUAGE:CUDA>:-O3>
  )
else()
  add_library(evs_benchmark_gpu src/cuda_backend_stub.cpp)
  target_include_directories(evs_benchmark_gpu PUBLIC include)
endif()

add_executable(evs_bench src/main.cpp)
target_link_libraries(evs_bench PRIVATE evs_benchmark_core evs_benchmark_gpu)
target_compile_options(evs_bench PRIVATE -Wall -Wextra -Wpedantic)

if(EVS_BENCH_ENABLE_METAVISION)
  find_package(MetavisionSDK COMPONENTS base stream QUIET)
  if(MetavisionSDK_FOUND)
    add_executable(evs_raw_to_evbin src/metavision_convert.cpp)
    target_link_libraries(evs_raw_to_evbin PRIVATE
      evs_benchmark_core
      MetavisionSDK::base
      MetavisionSDK::stream
    )
    target_compile_options(evs_raw_to_evbin PRIVATE -Wall -Wextra -Wpedantic)
    add_executable(evs_raw_decode_bench src/metavision_decode_benchmark.cpp)
    target_link_libraries(evs_raw_decode_bench PRIVATE
      MetavisionSDK::base
      MetavisionSDK::stream
    )
    target_compile_options(evs_raw_decode_bench PRIVATE -Wall -Wextra -Wpedantic)
  else()
    message(STATUS "MetavisionSDK not found: evs_raw_to_evbin will not be built")
  endif()
endif()

include(CTest)
if(BUILD_TESTING)
  add_executable(evs_benchmark_test tests/test_cpu.cpp)
  target_link_libraries(evs_benchmark_test PRIVATE evs_benchmark_core)
  add_test(NAME evs_benchmark_cpu_correctness COMMAND evs_benchmark_test)
endif()
