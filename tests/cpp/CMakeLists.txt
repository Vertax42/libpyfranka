cmake_minimum_required(VERSION 3.16.3)

# GTest via FetchContent (no system dependency required).
include(FetchContent)
FetchContent_Declare(googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG v1.14.0
)
FetchContent_MakeAvailable(googletest)

add_executable(test_trajectory test_trajectory.cpp)
target_include_directories(test_trajectory PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(test_trajectory PRIVATE gtest_main)
add_test(NAME test_trajectory COMMAND test_trajectory)

add_executable(test_rt_safety test_rt_safety.cpp)
target_include_directories(test_rt_safety PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(test_rt_safety PRIVATE gtest_main)
add_test(NAME test_rt_safety COMMAND test_rt_safety)

add_executable(test_ring_buffer test_ring_buffer.cpp)
target_include_directories(test_ring_buffer PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(test_ring_buffer PRIVATE gtest_main Threads::Threads)
add_test(NAME test_ring_buffer COMMAND test_ring_buffer)
