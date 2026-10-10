cmake_minimum_required(VERSION 3.20)
project(swerve_mppi_core VERSION 0.25.0 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(SWERVE_MPPI_CHASSIS_SOURCE_DIR "" CACHE PATH "Companion Python chassis checkout for offline parity")

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
    feedback/motion common/configuration common/timing adapter/current_chassis model/model planning/optimizer navigation/navigation
    safety/safety safety/admission planning/command
  )
    get_filename_component(suite "${suite_path}" NAME)
    add_executable(swerve_mppi_${suite}_tests tests/${suite_path}_tests.cpp)
    target_link_libraries(swerve_mppi_${suite}_tests PRIVATE swerve_mppi_core)
    target_include_directories(swerve_mppi_${suite}_tests PRIVATE src tests/fixtures)
    add_test(NAME ${suite}_regressions COMMAND swerve_mppi_${suite}_tests)
  endforeach()
  if(SWERVE_MPPI_CHASSIS_SOURCE_DIR)
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    add_executable(swerve_mppi_chassis_parity_driver tests/model/chassis_parity_driver.cpp)
    target_link_libraries(swerve_mppi_chassis_parity_driver PRIVATE swerve_mppi_core)
    add_test(NAME chassis_python_parity
      COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/chassis_parity_test.py
        --tool $<TARGET_FILE:swerve_mppi_chassis_parity_driver>
        --chassis-root ${SWERVE_MPPI_CHASSIS_SOURCE_DIR})
    set_tests_properties(chassis_python_parity PROPERTIES TIMEOUT 60)
  endif()
  add_executable(swerve_mppi_behavior_tests tests/integration/behavior_tests.cpp)
  target_link_libraries(swerve_mppi_behavior_tests PRIVATE swerve_mppi_core)
  target_include_directories(swerve_mppi_behavior_tests PRIVATE src tests/fixtures)
  foreach(scenario
    straight lateral curve spin reverse final_yaw scurve scurve_duplicates
    cusp loop short_cusp short_loop short_corner near_obstacles spin_translation
  )
    add_test(NAME behavior_${scenario} COMMAND swerve_mppi_behavior_tests ${scenario})
    set_tests_properties(behavior_${scenario} PROPERTIES TIMEOUT 180)
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
    swerve_mppi_tests motion_regressions configuration_regressions timing_regressions current_chassis_regressions model_regressions optimizer_regressions
    navigation_regressions safety_regressions admission_regressions command_regressions installed_consumer
    PROPERTIES TIMEOUT 60
  )
  target_compile_definitions(swerve_mppi_configuration_tests PRIVATE
    SWERVE_MPPI_DEFAULT_PROFILE="${CMAKE_CURRENT_SOURCE_DIR}/config/default.conf")
endif()

if(SWERVE_MPPI_BUILD_BENCHMARKS)
  add_executable(swerve_mppi_model_probe benchmarks/model_probe.cpp)
  target_include_directories(swerve_mppi_model_probe PRIVATE tests/fixtures)
  target_link_libraries(swerve_mppi_model_probe PRIVATE swerve_mppi_core)
  add_executable(swerve_mppi_benchmark benchmarks/core_benchmark.cpp)
  target_include_directories(swerve_mppi_benchmark PRIVATE src tests/fixtures)
  target_link_libraries(swerve_mppi_benchmark PRIVATE swerve_mppi_core)
  if(SWERVE_MPPI_BUILD_TESTS)
    add_test(NAME benchmark_allocations COMMAND swerve_mppi_benchmark --smoke)
    set_tests_properties(benchmark_allocations PROPERTIES TIMEOUT 60)
    if(Python3_Interpreter_FOUND)
      add_test(NAME motion_probe_cli
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/motion_probe_test.py
          --tool $<TARGET_FILE:swerve_mppi_model_probe>
          --summary ${CMAKE_CURRENT_SOURCE_DIR}/tools/summarize_motion_probe.py)
      set_tests_properties(motion_probe_cli PROPERTIES TIMEOUT 60)
    endif()
  endif()
endif()

install(TARGETS swerve_mppi_core EXPORT swerve_mppiTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES config/default.conf DESTINATION ${CMAKE_INSTALL_DATADIR}/swerve_mppi)

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
