cmake_minimum_required(VERSION 3.14)

include(cmake/prelude.cmake)

project(
    libcurves
    VERSION 0.1.0
    DESCRIPTION "Short description"
    HOMEPAGE_URL "https://example.com/"
    LANGUAGES NONE
)

include(cmake/project-is-top-level.cmake)
include(cmake/variables.cmake)

# ---- Declare library ----

add_library(libcurves_libcurves INTERFACE)
add_library(libcurves::libcurves ALIAS libcurves_libcurves)

set_property(
    TARGET libcurves_libcurves PROPERTY
    EXPORT_NAME libcurves
)

target_include_directories(
    libcurves_libcurves ${warning_guard}
    INTERFACE
    "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
)

target_compile_features(libcurves_libcurves INTERFACE cxx_std_20)

find_package(Boost 1.82.0 REQUIRED COMPONENTS math)
target_link_libraries(libcurves_libcurves INTERFACE Boost::math)

# ---- Install rules ----

if(NOT CMAKE_SKIP_INSTALL_RULES)
  include(cmake/install-rules.cmake)
endif()

# ---- Developer mode ----

if(NOT libcurves_DEVELOPER_MODE)
  return()
elseif(NOT PROJECT_IS_TOP_LEVEL)
  message(
      AUTHOR_WARNING
      "Developer mode is intended for developers of libcurves"
  )
endif()

include(cmake/dev-mode.cmake)
