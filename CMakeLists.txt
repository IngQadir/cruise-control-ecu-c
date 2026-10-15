cmake_minimum_required(VERSION 3.10)
project(cruise_control C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

option(WARNINGS_AS_ERRORS "Treat compiler warnings in src/ as errors" ON)
option(ENABLE_COVERAGE    "Build with gcov coverage instrumentation"  OFF)

# ---------------------------------------------------------------------------
# Strict warnings for production code (src/), applied per target
# ---------------------------------------------------------------------------
function(set_strict_warnings target)
    target_compile_options(${target} PRIVATE
        -Wall -Wextra -Wconversion -Wshadow -Wpedantic)
    if(WARNINGS_AS_ERRORS)
        target_compile_options(${target} PRIVATE -Werror)
    endif()
endfunction()

# ---------------------------------------------------------------------------
# Production library: the three ECU software modules
# ---------------------------------------------------------------------------
add_library(cc_lib STATIC
    src/pi_controller.c
    src/cc_statemachine.c
    src/can_signals.c)
target_include_directories(cc_lib PUBLIC src)
set_strict_warnings(cc_lib)

if(ENABLE_COVERAGE)
    target_compile_options(cc_lib PRIVATE --coverage -O0 -g)
    target_link_libraries(cc_lib PUBLIC --coverage)
endif()

# ---------------------------------------------------------------------------
# Shared library for Python (ctypes): build/libcc.so
# Same three modules, so Python drives the real ECU code.
# ---------------------------------------------------------------------------
add_library(cc_shared SHARED
    src/pi_controller.c
    src/cc_statemachine.c
    src/can_signals.c)
set_target_properties(cc_shared PROPERTIES OUTPUT_NAME cc)
target_include_directories(cc_shared PUBLIC src)
set_strict_warnings(cc_shared)


# ---------------------------------------------------------------------------
# Demo application
# ---------------------------------------------------------------------------
add_executable(cruise_demo src/main.c)
target_link_libraries(cruise_demo PRIVATE cc_lib m)
set_strict_warnings(cruise_demo)

# ---------------------------------------------------------------------------
# Unit tests (Unity)
# ---------------------------------------------------------------------------
enable_testing()

add_library(unity STATIC test/unity/unity.c)
target_include_directories(unity PUBLIC test/unity)

function(add_unity_test name)
    add_executable(${name} test/${name}.c)
    target_link_libraries(${name} PRIVATE cc_lib unity m)
    target_compile_options(${name} PRIVATE -Wall -Wextra)
    add_test(NAME ${name} COMMAND ${name})
endfunction()

add_unity_test(test_pi_controller)
add_unity_test(test_statemachine)
add_unity_test(test_can_signals)

# ---------------------------------------------------------------------------
# Static analysis: cmake --build build --target cppcheck
# ---------------------------------------------------------------------------
find_program(CPPCHECK_EXE cppcheck)
if(CPPCHECK_EXE)
    # unusedFunction / staticFunction are suppressed because the public API
    # functions are called from test/, which is not part of this analysis.
    # unmatchedSuppression: different cppcheck versions report different
    # checks, so a suppression that matches nothing must not fail the build.
    add_custom_target(cppcheck
        COMMAND ${CPPCHECK_EXE}
                --enable=all --inconclusive --std=c99
                --check-level=exhaustive
                --error-exitcode=1
                --suppress=missingIncludeSystem
                --suppress=unusedFunction
                --suppress=staticFunction
                --suppress=checkersReport
                --suppress=unmatchedSuppression
                -I ${CMAKE_SOURCE_DIR}/src
                ${CMAKE_SOURCE_DIR}/src
        COMMENT "Running cppcheck on src/"
        VERBATIM)
endif()

