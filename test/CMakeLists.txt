cmake_minimum_required(VERSION 3.14)

project(libcurvesTests LANGUAGES CXX)

include(../cmake/project-is-top-level.cmake)
include(../cmake/folders.cmake)

# ---- Dependencies ----

if(PROJECT_IS_TOP_LEVEL)
  find_package(libcurves REQUIRED)
  enable_testing()
endif()

find_package(Catch2 REQUIRED)
include(Catch)

# ---- Tests ----

add_executable(libcurves_test source/libcurves_test.cpp)
target_link_libraries(
    libcurves_test PRIVATE
    libcurves::libcurves
    Catch2::Catch2WithMain
)
target_compile_features(libcurves_test PRIVATE cxx_std_20)

catch_discover_tests(libcurves_test)

# ---- End-of-file commands ----

add_folders(Test)
