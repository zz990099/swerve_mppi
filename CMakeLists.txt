cmake_minimum_required(VERSION 3.20)
project(swerve_mppi_core VERSION 0.4.0 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

option(SWERVE_MPPI_BUILD_TESTS "Build standalone tests" ON)

add_library(swerve_mppi_core
  src/config.cpp
  src/kinematics.cpp
  src/model.cpp
  src/transition_model.cpp
  src/rollout.cpp
  src/critics.cpp
  src/noise.cpp
  src/mode_scheduler.cpp
  src/mode_manager.cpp
  src/executor.cpp
  src/optimizer.cpp
  src/controller.cpp
)
add_library(swerve_mppi::core ALIAS swerve_mppi_core)
set_target_properties(swerve_mppi_core PROPERTIES EXPORT_NAME core)
target_include_directories(swerve_mppi_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_compile_features(swerve_mppi_core PUBLIC cxx_std_17)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(swerve_mppi_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(SWERVE_MPPI_BUILD_TESTS)
  enable_testing()
  add_executable(swerve_mppi_tests tests/core_tests.cpp)
  target_link_libraries(swerve_mppi_tests PRIVATE swerve_mppi_core)
  add_test(NAME swerve_mppi_tests COMMAND swerve_mppi_tests)
  foreach(suite model optimizer execution)
    add_executable(swerve_mppi_${suite}_tests tests/${suite}_tests.cpp)
    target_link_libraries(swerve_mppi_${suite}_tests PRIVATE swerve_mppi_core)
    add_test(NAME ${suite}_regressions COMMAND swerve_mppi_${suite}_tests)
  endforeach()
  add_executable(swerve_mppi_behavior_tests tests/behavior_tests.cpp)
  target_link_libraries(swerve_mppi_behavior_tests PRIVATE swerve_mppi_core)
  foreach(scenario straight lateral curve spin)
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
  set_tests_properties(swerve_mppi_tests model_regressions optimizer_regressions execution_regressions
    installed_consumer PROPERTIES TIMEOUT 60)
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
