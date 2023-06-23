# Project is configured with no languages, so tell GNUInstallDirs the lib dir
set(CMAKE_INSTALL_LIBDIR lib CACHE PATH "")

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

# find_package(<package>) call for consumers to find this project
set(package libcurves)

install(
    DIRECTORY include/
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
    COMPONENT libcurves_Development
)

install(
    TARGETS libcurves_libcurves
    EXPORT libcurvesTargets
    INCLUDES DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
)

write_basic_package_version_file(
    "${package}ConfigVersion.cmake"
    COMPATIBILITY SameMajorVersion
    ARCH_INDEPENDENT
)

# Allow package maintainers to freely override the path for the configs
set(
    libcurves_INSTALL_CMAKEDIR "${CMAKE_INSTALL_DATADIR}/${package}"
    CACHE PATH "CMake package config location relative to the install prefix"
)
mark_as_advanced(libcurves_INSTALL_CMAKEDIR)

install(
    FILES cmake/install-config.cmake
    DESTINATION "${libcurves_INSTALL_CMAKEDIR}"
    RENAME "${package}Config.cmake"
    COMPONENT libcurves_Development
)

install(
    FILES "${PROJECT_BINARY_DIR}/${package}ConfigVersion.cmake"
    DESTINATION "${libcurves_INSTALL_CMAKEDIR}"
    COMPONENT libcurves_Development
)

install(
    EXPORT libcurvesTargets
    NAMESPACE libcurves::
    DESTINATION "${libcurves_INSTALL_CMAKEDIR}"
    COMPONENT libcurves_Development
)

if(PROJECT_IS_TOP_LEVEL)
  include(CPack)
endif()
