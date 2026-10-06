file(MAKE_DIRECTORY "${TEST_DIR}")
set(profile "${TEST_DIR}/input.conf")
set(resolved "${TEST_DIR}/resolved.conf")
file(WRITE "${profile}" "dt_s=.05\ncompute_budget_ratio=0\nhorizon_steps=2\nsamples_per_branch=1\niterations=1\nrandom_seed=7\n")

function(expect_rejection diagnostic)
  execute_process(COMMAND "${BUDGET_TOOL}" 2 ${ARGN}
    RESULT_VARIABLE result OUTPUT_VARIABLE output ERROR_VARIABLE error)
  if(NOT result EQUAL 1 OR NOT error MATCHES "${diagnostic}" OR NOT output STREQUAL "")
    message(FATAL_ERROR "Expected rejection ${diagnostic}: ${result}; ${output}; ${error}")
  endif()
endfunction()

expect_rejection("live configuration" --config "${profile}")
expect_rejection("cannot read configuration" --config "${TEST_DIR}/missing.conf")
expect_rejection("usage" --config)
file(WRITE "${TEST_DIR}/bad.conf" "unknown=1\n")
expect_rejection("configuration line 1" --config "${TEST_DIR}/bad.conf")
file(WRITE "${TEST_DIR}/limits.conf" "max_obstacles=1\n")
expect_rejection("workload exceeds" --config "${TEST_DIR}/limits.conf")
string(REPEAT "#" 65537 oversized)
file(WRITE "${TEST_DIR}/oversized.conf" "${oversized}")
expect_rejection("64 KiB" --config "${TEST_DIR}/oversized.conf")

# Explicit ratio wins even when the profile appears later in argv. The model
# interval and seed must still come from the file; this exercises all three
# pipeline consumers rather than only parsing an unused configuration.
execute_process(COMMAND "${BUDGET_TOOL}" 2 --budget-ratio .6 --config "${profile}"
  --write-config "${resolved}"
  RESULT_VARIABLE result OUTPUT_VARIABLE output ERROR_VARIABLE error)
if(NOT result EQUAL 0 OR NOT output MATCHES "41,128,2,30," OR NOT error STREQUAL "")
  message(FATAL_ERROR "Configured pipeline failed: ${result}; ${output}; ${error}")
endif()
file(READ "${resolved}" text)
if(NOT text MATCHES "random_seed = 7 " OR NOT text MATCHES "compute_budget_ratio = 0.599")
  message(FATAL_ERROR "Resolved profile lost actual overrides: ${text}")
endif()
execute_process(COMMAND "${BUDGET_TOOL}" 2 --config "${resolved}"
  RESULT_VARIABLE result OUTPUT_VARIABLE output ERROR_VARIABLE error)
if(NOT result EQUAL 0 OR NOT output MATCHES "41,128,2,30,")
  message(FATAL_ERROR "Resolved profile cannot reproduce pipeline: ${result}; ${output}; ${error}")
endif()
