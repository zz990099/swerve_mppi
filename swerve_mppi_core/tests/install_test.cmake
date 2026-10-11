function(run_checked)
  execute_process(COMMAND ${ARGV} RESULT_VARIABLE result)
  if(NOT result EQUAL 0)
    message(FATAL_ERROR "Installed consumer command failed: ${ARGV}")
  endif()
endfunction()
file(REMOVE_RECURSE "${INSTALL_TEST_DIR}")
run_checked("${CMAKE_COMMAND}" --install "${PROJECT_BUILD_DIR}"
  --prefix "${INSTALL_TEST_DIR}/prefix" --config "${TEST_CONFIG}")
# A relocated install must expose only the current package identity and must
# not retain references to the original source or install prefix.
file(RENAME "${INSTALL_TEST_DIR}/prefix" "${INSTALL_TEST_DIR}/relocated")
file(GLOB_RECURSE obsolete_configs
  "${INSTALL_TEST_DIR}/relocated/*/swerve_mppiConfig*.cmake"
  "${INSTALL_TEST_DIR}/relocated/*/swerve_mppiTargets*.cmake")
if(obsolete_configs)
  message(FATAL_ERROR "Removed package aliases were installed: ${obsolete_configs}")
endif()
if(NOT EXISTS "${INSTALL_TEST_DIR}/relocated/share/swerve_mppi_core/package.xml" OR
   NOT EXISTS "${INSTALL_TEST_DIR}/relocated/share/swerve_mppi_core/default.conf")
  message(FATAL_ERROR "Core package metadata or default configuration was not installed")
endif()
run_checked("${CMAKE_COMMAND}" -S "${CONSUMER_SOURCE_DIR}"
  -B "${INSTALL_TEST_DIR}/consumer" -G "${TEST_GENERATOR}"
  "-DCMAKE_PREFIX_PATH=${INSTALL_TEST_DIR}/relocated"
  "-DCMAKE_CXX_COMPILER=${TEST_COMPILER}" "-DCMAKE_BUILD_TYPE=${TEST_CONFIG}")
run_checked("${CMAKE_COMMAND}" --build "${INSTALL_TEST_DIR}/consumer"
  --config "${TEST_CONFIG}" --parallel 2)
run_checked("${CMAKE_CTEST_COMMAND}" --test-dir "${INSTALL_TEST_DIR}/consumer"
  -C "${TEST_CONFIG}" --output-on-failure)
