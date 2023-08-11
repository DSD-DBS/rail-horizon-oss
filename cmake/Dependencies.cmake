# SPDX-FileCopyrightText: Copyright DB Netz AG
# SPDX-License-Identifier: CC0-1.0

# Private helper method to separate one option string like "Name Test" in key and value variables
function(_parse_option OPTION)
    string(REGEX MATCH "^[^ ]+" OPTION_KEY "${OPTION}")
    string(LENGTH "${OPTION}" OPTION_LENGTH)
    string(LENGTH "${OPTION_KEY}" OPTION_KEY_LENGTH)
    if(OPTION_KEY_LENGTH STREQUAL OPTION_LENGTH)
        # no value for key provided, assume user wants to set option to "ON"
        set(OPTION_VALUE "ON")
    else()
        math(EXPR OPTION_KEY_LENGTH "${OPTION_KEY_LENGTH}+1")
        string(SUBSTRING "${OPTION}" "${OPTION_KEY_LENGTH}" "-1" OPTION_VALUE)
    endif()
    set(OPTION_KEY
        "${OPTION_KEY}"
        PARENT_SCOPE
    )
    set(OPTION_VALUE
        "${OPTION_VALUE}"
        PARENT_SCOPE
    )
endfunction()

# Function to add a submodule to a project
function(add_submodule)
    set(OPTIONS SYSTEM)
    set(ONE_VALUE_ARGS NAME BINARY_BASE_DIR)
    set(MULTI_VALUE_ARGS OPTIONS)
    cmake_parse_arguments(ARGS "${OPTIONS}" "${ONE_VALUE_ARGS}" "${MULTI_VALUE_ARGS}" ${ARGN})

    foreach(OPTION ${ARGS_OPTIONS})
        _parse_option("${OPTION}")
        set(${OPTION_KEY} "${OPTION_VALUE}")
    endforeach()

    if(NOT DEFINED ARGS_NAME)
        message(FATAL_ERROR "BINARY_BASE_DIR must be specified on add_submodule call")
    endif()

    if(NOT DEFINED ARGS_BINARY_BASE_DIR)
        message(FATAL_ERROR "BINARY_BASE_DIR must be specified on add_submodule call")
    endif()

    set(DEPENDENCY_SOURCE_DIR "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/../submodules/${ARGS_NAME}")
    set(DEPENDENCY_BINARY_DIR "${ARGS_BINARY_BASE_DIR}/submodules/${ARGS_NAME}")
    if(DEFINED ARGS_SYSTEM)
        add_subdirectory(${DEPENDENCY_SOURCE_DIR} ${DEPENDENCY_BINARY_DIR} SYSTEM)
    else()
        add_subdirectory(${DEPENDENCY_SOURCE_DIR} ${DEPENDENCY_BINARY_DIR})
    endif()
endfunction()

# Function to add a google submodule to a project. Multiple ros packages will reuse the same googletest build.
function(add_googletest_submodule)
    add_submodule(
        NAME "external/googletest"
        BINARY_BASE_DIR "${PROJECT_BINARY_DIR}/.."
        OPTIONS "INSTALL_GTEST OFF"
    )
endfunction()
