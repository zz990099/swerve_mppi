function(run_checked)
  execute_process(COMMAND ${ARGV} RESULT_VARIABLE result)
  if(NOT result EQUAL 0)
    message(FATAL_ERROR "Installed consumer command failed: ${ARGV}")
  endif()
endfunction()
file(REMOVE_RECURSE "${INSTALL_TEST_DIR}")
run_checked("${CMAKE_COMMAND}" --install "${PROJECT_BUILD_DIR}"
  --prefix "${INSTALL_TEST_DIR}/prefix" --config "${TEST_CONFIG}")
run_checked("${CMAKE_COMMAND}" -S "${CONSUMER_SOURCE_DIR}"
  -B "${INSTALL_TEST_DIR}/consumer" -G "${TEST_GENERATOR}"
  "-DCMAKE_PREFIX_PATH=${INSTALL_TEST_DIR}/prefix"
  "-DCMAKE_CXX_COMPILER=${TEST_COMPILER}" "-DCMAKE_BUILD_TYPE=${TEST_CONFIG}")
run_checked("${CMAKE_COMMAND}" --build "${INSTALL_TEST_DIR}/consumer"
  --config "${TEST_CONFIG}")
run_checked("${CMAKE_CTEST_COMMAND}" --test-dir "${INSTALL_TEST_DIR}/consumer"
  -C "${TEST_CONFIG}" --output-on-failure)
