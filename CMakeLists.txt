cmake_minimum_required(VERSION 3.16)

project(obs-dynamic-autocrop
    VERSION     1.0.0
    DESCRIPTION "Dynamic auto-crop filter for retro upscaler / capture card footage"
    LANGUAGES   C
)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED TRUE)

# ── Find OBS ─────────────────────────────────────────────────────────────────
#
# First try the standard find_package path (works if OBS ships cmake configs).
# If that fails, fall back to manually specified OBS_INCLUDE_DIR + OBS_LIB
# which the build.bat script sets up by downloading OBS source headers and
# generating an import library from the installed obs.dll.
#
find_package(libobs QUIET)

if(NOT TARGET OBS::libobs)
    message(STATUS "find_package(libobs) did not find a config file, "
                   "falling back to OBS_INCLUDE_DIR / OBS_LIB")

    if(NOT OBS_INCLUDE_DIR OR NOT OBS_LIB)
        message(FATAL_ERROR
            "Could not locate OBS development files.\n"
            "Run build.bat — it will download them automatically.\n"
            "Or set -DOBS_INCLUDE_DIR=<path> and -DOBS_LIB=<path> manually.")
    endif()

    add_library(OBS::libobs UNKNOWN IMPORTED)
    set_target_properties(OBS::libobs PROPERTIES
        IMPORTED_LOCATION             "${OBS_LIB}"
        INTERFACE_INCLUDE_DIRECTORIES "${OBS_INCLUDE_DIR}"
    )
endif()

# ── Plugin target ─────────────────────────────────────────────────────────────
add_library(obs-dynamic-autocrop MODULE
    src/dynamic_autocrop_filter.c
)

target_link_libraries(obs-dynamic-autocrop
    PRIVATE OBS::libobs
)

if(MSVC)
    target_compile_options(obs-dynamic-autocrop PRIVATE /W3)
else()
    target_compile_options(obs-dynamic-autocrop PRIVATE -Wall -Wextra -Wno-unused-parameter)
endif()

# ── Install ───────────────────────────────────────────────────────────────────
if(WIN32)
    set_target_properties(obs-dynamic-autocrop PROPERTIES PREFIX "" SUFFIX ".dll")
    install(TARGETS obs-dynamic-autocrop
        RUNTIME DESTINATION "obs-plugins/64bit"
        LIBRARY DESTINATION "obs-plugins/64bit"
    )
    install(DIRECTORY data/
        DESTINATION "data/obs-plugins/obs-dynamic-autocrop"
    )
elseif(APPLE)
    set_target_properties(obs-dynamic-autocrop PROPERTIES
        PREFIX "" SUFFIX ".so"
        BUILD_WITH_INSTALL_RPATH TRUE
        INSTALL_RPATH "@loader_path/../.."
    )
    install(TARGETS obs-dynamic-autocrop LIBRARY DESTINATION "obs-plugins")
    install(DIRECTORY data/ DESTINATION "data/obs-plugins/obs-dynamic-autocrop")
else()
    include(GNUInstallDirs)
    set_target_properties(obs-dynamic-autocrop PROPERTIES PREFIX "" SUFFIX ".so")
    install(TARGETS obs-dynamic-autocrop
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}/obs-plugins"
    )
    install(DIRECTORY data/
        DESTINATION "${CMAKE_INSTALL_DATAROOTDIR}/obs-plugins/obs-dynamic-autocrop"
    )
endif()
