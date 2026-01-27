cmake_minimum_required(VERSION 3.15)
project(starSense)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Default to Release build if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Build flags
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# ----------------------------------------------------------
# Get pybind11 and Python include paths from current Python
# ----------------------------------------------------------
execute_process(
    COMMAND python3 -m pybind11 --includes
    OUTPUT_VARIABLE PYBIND11_INCLUDES_RAW
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

execute_process(
    COMMAND python3-config --includes
    OUTPUT_VARIABLE PYTHON_INCLUDES_RAW
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Combine and parse the include paths
set(ALL_INCLUDE_FLAGS "${PYBIND11_INCLUDES_RAW} ${PYTHON_INCLUDES_RAW}")
separate_arguments(ALL_INCLUDE_FLAGS)
set(INCLUDE_DIRS "")
foreach(flag ${ALL_INCLUDE_FLAGS})
    if(flag MATCHES "^-I(.+)")
        list(APPEND INCLUDE_DIRS "${CMAKE_MATCH_1}")
    endif()
endforeach()

message(STATUS "Detected Python/pybind11 include dirs:")
foreach(dir ${INCLUDE_DIRS})
    message(STATUS "  ${dir}")
endforeach()

# ----------------------------------------------------------
# Add all C++ source files
# ----------------------------------------------------------
file(GLOB_RECURSE SOURCES
    cpp/core/*.cpp
    cpp/interface/*.cpp
)

# ----------------------------------------------------------
# Create the Python extension module
# ----------------------------------------------------------
add_library(starSense MODULE ${SOURCES})

target_include_directories(starSense
    PRIVATE
        cpp/core
        cpp/interface
        ${INCLUDE_DIRS}
)

# macOS linker: allow unresolved Python symbols
if(APPLE)
    set_target_properties(starSense PROPERTIES
        LINK_FLAGS "-undefined dynamic_lookup"
    )
endif()

# Remove 'lib' prefix for Python import
set_target_properties(starSense PROPERTIES PREFIX "" OUTPUT_NAME "starSense")
