cmake_minimum_required(VERSION 3.16.3)
project(libfranka_robot_diagnostics LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT TARGET Franka::Franka)
  find_package(Franka 0.21 REQUIRED)
endif()
find_package(Threads REQUIRED)

add_executable(libfranka_cartesian_sine libfranka_cartesian_sine.cpp)
target_link_libraries(libfranka_cartesian_sine PRIVATE
    Franka::Franka
    Threads::Threads
)

# libfranka may have transitive dependencies (notably libfmt) outside the
# system loader path. Use DT_RPATH rather than DT_RUNPATH so those transitive
# dependencies are found without asking the operator to set LD_LIBRARY_PATH.
target_link_options(libfranka_cartesian_sine PRIVATE "-Wl,--disable-new-dtags")
