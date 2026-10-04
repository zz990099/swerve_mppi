cmake_minimum_required(VERSION 3.20)
project(swerve_mppi_core VERSION 0.20.2 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

option(SWERVE_MPPI_BUILD_TESTS "Build standalone tests" ON)
option(SWERVE_MPPI_BUILD_BENCHMARKS "Build standalone measurement tools" OFF)
option(SWERVE_MPPI_BUILD_FORMAT_TESTS "Check the pinned ROS Rolling C++ format" OFF)

if(SWERVE_MPPI_BUILD_FORMAT_TESTS)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
else()
  find_package(Python3 QUIET COMPONENTS Interpreter)
endif()
if(Python3_Interpreter_FOUND)
  add_custom_target(format
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tools/format.py --fix
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Apply the official ROS Rolling format")
  add_custom_target(format_check
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tools/format.py --check
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Check the official ROS Rolling format")
endif()
if(SWERVE_MPPI_BUILD_FORMAT_TESTS)
  enable_testing()
  add_test(NAME ros_format
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tools/format.py --check)
  set_tests_properties(ros_format PROPERTIES TIMEOUT 60)
endif()

add_library(swerve_mppi_core)
add_subdirectory(src)
add_library(swerve_mppi::core ALIAS swerve_mppi_core)
set_target_properties(swerve_mppi_core PROPERTIES EXPORT_NAME core POSITION_INDEPENDENT_CODE ON)
target_include_directories(swerve_mppi_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src"
)
target_compile_features(swerve_mppi_core PUBLIC cxx_std_17)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(swerve_mppi_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(SWERVE_MPPI_BUILD_TESTS)
  enable_testing()
  add_executable(swerve_mppi_tests tests/common/core_tests.cpp)
  target_link_libraries(swerve_mppi_tests PRIVATE swerve_mppi_core)
  target_include_directories(swerve_mppi_tests PRIVATE src tests/fixtures)
  add_test(NAME swerve_mppi_tests COMMAND swerve_mppi_tests)
  foreach(suite_path
    model/model planning/optimizer execution/execution navigation/navigation
    safety/safety execution/timing model/actuation safety/admission integration/integration
    execution/chassis
  )
    get_filename_component(suite "${suite_path}" NAME)
    add_executable(swerve_mppi_${suite}_tests tests/${suite_path}_tests.cpp)
    target_link_libraries(swerve_mppi_${suite}_tests PRIVATE swerve_mppi_core)
    target_include_directories(swerve_mppi_${suite}_tests PRIVATE src tests/fixtures)
    add_test(NAME ${suite}_regressions COMMAND swerve_mppi_${suite}_tests)
  endforeach()
  add_executable(swerve_mppi_behavior_tests tests/integration/behavior_tests.cpp)
  target_link_libraries(swerve_mppi_behavior_tests PRIVATE swerve_mppi_core)
  target_include_directories(swerve_mppi_behavior_tests PRIVATE tests/fixtures)
  foreach(scenario
    straight lateral curve spin reverse final_yaw scurve scurve_duplicates
    cusp loop short_cusp short_loop short_corner near_obstacles spin_translation
  )
    add_test(NAME behavior_${scenario} COMMAND swerve_mppi_behavior_tests ${scenario})
    add_test(NAME timed_behavior_${scenario} COMMAND swerve_mppi_behavior_tests ${scenario} --timed)
    add_test(NAME profile_behavior_${scenario} COMMAND swerve_mppi_behavior_tests ${scenario} --profile)
    set_tests_properties(profile_behavior_${scenario} PROPERTIES TIMEOUT 180)
    set_tests_properties(behavior_${scenario} PROPERTIES TIMEOUT 180)
    set_tests_properties(timed_behavior_${scenario} PROPERTIES TIMEOUT 180)
  endforeach()
  add_test(NAME installed_consumer
    COMMAND ${CMAKE_COMMAND}
      -DPROJECT_BUILD_DIR=${CMAKE_CURRENT_BINARY_DIR}
      -DCONSUMER_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/tests/consumer
      -DINSTALL_TEST_DIR=${CMAKE_CURRENT_BINARY_DIR}/install-test
      -DTEST_CONFIG=$<CONFIG>
      -DTEST_GENERATOR=${CMAKE_GENERATOR}
      -DTEST_COMPILER=${CMAKE_CXX_COMPILER}
      -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/install_test.cmake)
  set_tests_properties(
    swerve_mppi_tests model_regressions optimizer_regressions execution_regressions
    navigation_regressions safety_regressions timing_regressions actuation_regressions
    admission_regressions integration_regressions chassis_regressions installed_consumer
    PROPERTIES TIMEOUT 60
  )
endif()

if(SWERVE_MPPI_BUILD_BENCHMARKS)
  add_executable(swerve_mppi_integration_budget benchmarks/integration_budget.cpp)
  target_include_directories(swerve_mppi_integration_budget PRIVATE tests/fixtures)
  target_link_libraries(swerve_mppi_integration_budget PRIVATE swerve_mppi_core)
  add_executable(swerve_mppi_benchmark benchmarks/core_benchmark.cpp)
  target_include_directories(swerve_mppi_benchmark PRIVATE tests/fixtures)
  target_link_libraries(swerve_mppi_benchmark PRIVATE swerve_mppi_core)
  if(SWERVE_MPPI_BUILD_TESTS)
    add_test(NAME benchmark_allocations COMMAND swerve_mppi_benchmark --smoke)
    set_tests_properties(benchmark_allocations PROPERTIES TIMEOUT 60)
  endif()
endif()

install(TARGETS swerve_mppi_core EXPORT swerve_mppiTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

set(swerve_mppi_package_dir "${CMAKE_INSTALL_LIBDIR}/cmake/swerve_mppi")
configure_package_config_file(
  cmake/swerve_mppiConfig.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/swerve_mppiConfig.cmake"
  INSTALL_DESTINATION "${swerve_mppi_package_dir}"
)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/swerve_mppiConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMinorVersion
)
install(EXPORT swerve_mppiTargets
  NAMESPACE swerve_mppi::
  DESTINATION "${swerve_mppi_package_dir}"
)
install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/swerve_mppiConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/swerve_mppiConfigVersion.cmake"
  DESTINATION "${swerve_mppi_package_dir}"
)
