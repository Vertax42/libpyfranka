cmake_minimum_required(VERSION 3.16.3)
project(libpyfranka VERSION 0.1.0)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

function(_libpyfranka_get_target_location out_var target_name)
    foreach(_prop
        IMPORTED_LOCATION_RELEASE
        IMPORTED_LOCATION_RELWITHDEBINFO
        IMPORTED_LOCATION_MINSIZEREL
        IMPORTED_LOCATION_DEBUG
        IMPORTED_LOCATION_NOCONFIG
        LOCATION
    )
        get_target_property(_loc "${target_name}" "${_prop}")
        if(_loc AND NOT _loc STREQUAL "_loc-NOTFOUND")
            set(${out_var} "${_loc}" PARENT_SCOPE)
            return()
        endif()
    endforeach()
    set(${out_var} "" PARENT_SCOPE)
endfunction()

function(_libpyfranka_extract_libfmt_soname out_var library_path)
    if(NOT EXISTS "${library_path}")
        set(${out_var} "" PARENT_SCOPE)
        return()
    endif()

    get_filename_component(_lib_name "${library_path}" NAME)
    string(REGEX MATCH "libfmt\\.so\\.([0-9]+)" _name_match "${_lib_name}")
    if(_name_match)
        set(${out_var} "${CMAKE_MATCH_0}" PARENT_SCOPE)
        return()
    endif()

    find_program(READELF_EXECUTABLE readelf)
    if(NOT READELF_EXECUTABLE)
        set(${out_var} "" PARENT_SCOPE)
        return()
    endif()

    execute_process(
        COMMAND "${READELF_EXECUTABLE}" -d "${library_path}"
        OUTPUT_VARIABLE _dynamic_section
        RESULT_VARIABLE _readelf_result
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    if(NOT _readelf_result EQUAL 0)
        set(${out_var} "" PARENT_SCOPE)
        return()
    endif()

    string(REGEX MATCH "SONAME[^[]*\\[(libfmt\\.so\\.[0-9]+)\\]" _soname_match "${_dynamic_section}")
    if(_soname_match)
        set(${out_var} "${CMAKE_MATCH_1}" PARENT_SCOPE)
        return()
    endif()

    string(REGEX MATCH "NEEDED[^[]*\\[(libfmt\\.so\\.[0-9]+)\\]" _needed_match "${_dynamic_section}")
    if(_needed_match)
        set(${out_var} "${CMAKE_MATCH_1}" PARENT_SCOPE)
        return()
    endif()

    set(${out_var} "" PARENT_SCOPE)
endfunction()

function(_libpyfranka_validate_franka_fmt_abi)
    if(NOT TARGET Franka::Franka OR NOT TARGET fmt::fmt)
        return()
    endif()

    _libpyfranka_get_target_location(_franka_lib Franka::Franka)
    if(NOT _franka_lib)
        message(WARNING "Could not resolve Franka::Franka library location; skipping fmt ABI validation.")
        return()
    endif()
    if(NOT EXISTS "${_franka_lib}")
        message(FATAL_ERROR "Franka::Franka resolved to a missing library: ${_franka_lib}")
    endif()

    find_program(READELF_EXECUTABLE readelf)
    if(NOT READELF_EXECUTABLE)
        message(WARNING "readelf not found; skipping fmt ABI validation for ${_franka_lib}.")
        return()
    endif()

    execute_process(
        COMMAND "${READELF_EXECUTABLE}" -d "${_franka_lib}"
        OUTPUT_VARIABLE _franka_dynamic
        RESULT_VARIABLE _franka_readelf_result
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    if(NOT _franka_readelf_result EQUAL 0)
        message(WARNING "readelf failed for ${_franka_lib}; skipping fmt ABI validation.")
        return()
    endif()

    string(REGEX MATCH "NEEDED[^[]*\\[(libfmt\\.so\\.[0-9]+)\\]" _franka_fmt_match "${_franka_dynamic}")
    if(NOT _franka_fmt_match)
        message(STATUS "libfranka does not advertise a dynamic fmt dependency; skipping fmt ABI validation.")
        return()
    endif()
    set(_franka_fmt_soname "${CMAKE_MATCH_1}")

    _libpyfranka_get_target_location(_fmt_lib fmt::fmt)
    if(NOT _fmt_lib)
        message(WARNING "Could not resolve fmt::fmt library location; skipping fmt ABI validation.")
        return()
    endif()

    _libpyfranka_extract_libfmt_soname(_fmt_soname "${_fmt_lib}")
    if(NOT _fmt_soname)
        message(WARNING "Could not determine fmt soname for ${_fmt_lib}; skipping fmt ABI validation.")
        return()
    endif()

    if(NOT "${_franka_fmt_soname}" STREQUAL "${_fmt_soname}")
        message(FATAL_ERROR
            "libfranka ABI mismatch: ${_franka_lib} needs ${_franka_fmt_soname}, "
            "but CMake resolved fmt::fmt to ${_fmt_lib} (${_fmt_soname}). "
            "Point CMAKE_PREFIX_PATH / fmt_DIR at a prefix containing the matching fmt build, "
            "or rebuild libfranka against the active environment. This is the same class of "
            "transitive fmt/spdlog mismatch handled in libpyflexiv."
        )
    endif()

    message(STATUS "libfranka fmt ABI check passed: ${_franka_fmt_soname} via ${_fmt_lib}")
endfunction()

# Find libfranka (resolved via CMAKE_PREFIX_PATH / Franka_DIR, e.g. ~/franka_install).
# This project targets libfranka 0.21.x APIs; older system installs such as
# /usr/local/libfranka 0.15 should be rejected during package resolution.
find_package(Franka 0.21 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

# Find Python and pybind11 (use Python's installed pybind11 to avoid version skew).
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
execute_process(
    COMMAND ${Python3_EXECUTABLE} -c
        "import pybind11; print(pybind11.get_cmake_dir())"
    OUTPUT_VARIABLE PYBIND11_CMAKE_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE _pybind11_result
)
if(NOT _pybind11_result EQUAL 0)
    message(FATAL_ERROR "Could not find pybind11. Install it with: pip install pybind11")
endif()
set(pybind11_DIR ${PYBIND11_CMAKE_DIR})
find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(_franka_rt
    src/franka_rt.cpp
    src/joint_position_control.cpp
    src/cartesian_control.cpp
)

target_include_directories(_franka_rt PRIVATE include/)
target_link_libraries(_franka_rt PRIVATE
    Franka::Franka
    Eigen3::Eigen
    Threads::Threads
)

# Mirror libpyflexiv's dependency hygiene: validate the transitive fmt ABI
# during configure instead of letting an import-time loader error surface later.
_libpyfranka_validate_franka_fmt_abi()

# GCC 13+ may emit -Wfree-nonheap-object on inlined std::vector usage.
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL "13")
    target_compile_options(_franka_rt PRIVATE -Wno-free-nonheap-object)
endif()

set_target_properties(_franka_rt PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/franka_rt
    OUTPUT_NAME "_franka_rt"
)

# Bake in RPATHs for libfranka and its transitive deps (pinocchio, eigenpy,
# etc.) so the extension can be loaded without LD_LIBRARY_PATH gymnastics.
# We pull the actual paths from the imported targets discovered above.
get_target_property(_franka_lib Franka::Franka LOCATION)
get_filename_component(_franka_libdir "${_franka_lib}" DIRECTORY)
set(_extra_rpaths "${_franka_libdir}")
_libpyfranka_get_target_location(_fmt_lib fmt::fmt)
if(_fmt_lib)
    get_filename_component(_fmt_libdir "${_fmt_lib}" DIRECTORY)
    list(APPEND _extra_rpaths "${_fmt_libdir}")
endif()

# pinocchio (and eigenpy / hpp-fcl) live alongside it under ROS humble.
if(EXISTS "/opt/ros/humble/lib")
    list(APPEND _extra_rpaths "/opt/ros/humble/lib")
endif()
if(EXISTS "/opt/ros/humble/lib/x86_64-linux-gnu")
    list(APPEND _extra_rpaths "/opt/ros/humble/lib/x86_64-linux-gnu")
endif()
list(REMOVE_DUPLICATES _extra_rpaths)

set_target_properties(_franka_rt PROPERTIES
    INSTALL_RPATH "${_extra_rpaths}"
    BUILD_RPATH "${_extra_rpaths}"
)
# DT_RPATH (legacy) propagates to transitive `dlopen`s — RUNPATH doesn't.
# This lets libfranka find pinocchio etc. without LD_LIBRARY_PATH at runtime.
target_link_options(_franka_rt PRIVATE "-Wl,--disable-new-dtags")

# Tests (no robot required for unit tests)
option(BUILD_TESTS "Build C++ unit tests" OFF)
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests/cpp)
endif()

# On-robot diagnostic binaries. These are intentionally not registered with
# ctest because running them moves a physical robot.
option(BUILD_ROBOT_TESTS "Build on-robot libfranka diagnostic tests" OFF)
if(BUILD_ROBOT_TESTS)
    add_subdirectory(tests/robot)
endif()
