cmake_minimum_required(VERSION 3.14)
project(tests)

# GoogleTest requires at least C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_MACOSX_RPATH 1)

include(FetchContent)
FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/release-1.11.0.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# adding the Google_Tests_run target
add_compile_options(-gdwarf-4)
add_executable(tests_run VDBVolumeTest.cpp)

# linking Google_Tests_run with vdbfusion which will be tested
target_link_libraries(tests_run vdbfusion)
target_link_libraries(tests_run gtest gtest_main)