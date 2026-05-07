cmake_minimum_required(VERSION 3.16.3)
project(libpyfranka VERSION 0.1.0)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find libfranka (resolved via CMAKE_PREFIX_PATH, e.g. ~/franka_install).
find_package(Franka REQUIRED)
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
    src/joint_torque_control.cpp
    src/cartesian_pose_control.cpp
    src/cartesian_impedance_control.cpp
    src/pose_math.cpp
)

target_include_directories(_franka_rt PRIVATE include/)
target_link_libraries(_franka_rt PRIVATE
    Franka::Franka
    Eigen3::Eigen
    Threads::Threads
)

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

# pinocchio (and eigenpy / hpp-fcl) live alongside it under ROS humble.
if(EXISTS "/opt/ros/humble/lib")
    list(APPEND _extra_rpaths "/opt/ros/humble/lib")
endif()
if(EXISTS "/opt/ros/humble/lib/x86_64-linux-gnu")
    list(APPEND _extra_rpaths "/opt/ros/humble/lib/x86_64-linux-gnu")
endif()

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
