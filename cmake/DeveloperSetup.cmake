# SPDX-FileCopyrightText: Copyright DB Netz AG
# SPDX-License-Identifier: CC0-1.0

option(WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)
option(ENABLE_COVERAGE "Enable coverage reporting for gcc/clang" ON)

# Enables coverage and compiler warnings for the given target
function(ipm_target_dev_setup TARGET)
    ipm_target_coverage(${TARGET})
    ipm_target_compile_warnings(${TARGET})
endfunction()

# Enables coverage for the given target
function(ipm_target_coverage TARGET)
    if(ENABLE_COVERAGE AND CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
        target_compile_options(${TARGET} PUBLIC -O0 -g --coverage)
        target_link_options(${TARGET} PUBLIC --coverage)
    endif()
endfunction()

# This function takes a cmake target as an argument and adds compiler warnings to it.
function(ipm_target_compile_warnings TARGET)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
        target_compile_options(${TARGET} PRIVATE -Wall -Wextra -Wpedantic)
    endif()
endfunction()
