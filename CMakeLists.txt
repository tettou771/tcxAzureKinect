# =============================================================================
# tcxAzureKinect - Azure Kinect DK backend for tcxDepthCamera
# =============================================================================
#
# Implements the tcxDepthCamera interface on top of the Azure Kinect Sensor SDK
# (k4a). The k4a SDK is officially supported on Windows and Linux only; this
# addon is not buildable on macOS.
#
# Finding k4a:
#   - Linux / vcpkg: a proper CMake package config (k4aConfig.cmake) usually
#     exists, so `find_package(k4a CONFIG)` provides the imported target
#     k4a::k4a directly.
#   - Windows: the official MSI installer ("Azure Kinect SDK vX.Y.Z") does NOT
#     ship a CMake config. We locate the headers + import library by hand under
#     the standard install dir and synthesize the k4a::k4a imported target, then
#     copy the runtime DLLs (k4a.dll + depthengine_*.dll) next to the app exe.
# =============================================================================

cmake_minimum_required(VERSION 3.16)

set(ADDON_NAME tcxAzureKinect)

if(APPLE)
    message(FATAL_ERROR
        "tcxAzureKinect: the Azure Kinect SDK (k4a) does not officially support "
        "macOS. Build on Linux or Windows.")
endif()

file(GLOB ADDON_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB ADDON_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h)

add_library(${ADDON_NAME} STATIC ${ADDON_SOURCES} ${ADDON_HEADERS})

# tcxDepthCamera is a header-only interface addon sitting next to us. Anchor its
# path at TRUSSC_DIR (like use_addon.cmake does) rather than a path relative to
# THIS dir: in CI the addon is symlinked into TrussC/addons/, and CMake resolves
# the symlink to its real location, so "../tcxDepthCamera" would miss (Linux).
if(DEFINED TRUSSC_DIR)
    set(_TC_DEPTHCAMERA_SRC "${TRUSSC_DIR}/../addons/tcxDepthCamera/src")
else()
    set(_TC_DEPTHCAMERA_SRC "${CMAKE_CURRENT_SOURCE_DIR}/../tcxDepthCamera/src")
endif()

target_include_directories(${ADDON_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${_TC_DEPTHCAMERA_SRC}
)

target_link_libraries(${ADDON_NAME} PUBLIC TrussC)

# -----------------------------------------------------------------------------
# Locate the Azure Kinect Sensor SDK (k4a).
# -----------------------------------------------------------------------------
# Prefer a real CMake package config when present (Linux distro / vcpkg). This
# defines the imported target k4a::k4a and, on Windows, would also carry DLL
# locations — so the manual fallback below is Windows-installer-only.
find_package(k4a QUIET CONFIG)

# Record DLLs to co-locate with the app exe (Windows). Filled by the manual
# path; left empty when find_package handled everything.
set(_TCAK_RUNTIME_DLLS "")

if(NOT TARGET k4a::k4a)
    if(WIN32)
        # Search the standard installer location, newest SDK first. Allow an
        # override via -DK4A_SDK_ROOT=... pointing at the "sdk" folder.
        set(_TCAK_SDK_HINTS "")
        if(DEFINED K4A_SDK_ROOT)
            list(APPEND _TCAK_SDK_HINTS "${K4A_SDK_ROOT}")
        endif()
        file(GLOB _TCAK_SDK_CANDIDATES
            "$ENV{ProgramFiles}/Azure Kinect SDK v*/sdk"
            "$ENV{ProgramW6432}/Azure Kinect SDK v*/sdk"
            "C:/Program Files/Azure Kinect SDK v*/sdk")
        # Newest version last from GLOB's lexical sort -> reverse so we try it first.
        list(SORT _TCAK_SDK_CANDIDATES)
        list(REVERSE _TCAK_SDK_CANDIDATES)
        list(APPEND _TCAK_SDK_HINTS ${_TCAK_SDK_CANDIDATES})

        set(_TCAK_INC_HINTS "")
        set(_TCAK_LIB_HINTS "")
        set(_TCAK_BIN_HINTS "")
        foreach(_root ${_TCAK_SDK_HINTS})
            list(APPEND _TCAK_INC_HINTS "${_root}/include")
            list(APPEND _TCAK_LIB_HINTS "${_root}/windows-desktop/amd64/release/lib")
            list(APPEND _TCAK_BIN_HINTS "${_root}/windows-desktop/amd64/release/bin")
        endforeach()

        find_path(K4A_INCLUDE_DIR NAMES k4a/k4a.h HINTS ${_TCAK_INC_HINTS})
        find_library(K4A_LIBRARY  NAMES k4a       HINTS ${_TCAK_LIB_HINTS})

        if(NOT K4A_INCLUDE_DIR OR NOT K4A_LIBRARY)
            message(FATAL_ERROR
                "tcxAzureKinect: could not find the Azure Kinect Sensor SDK.\n"
                "  Install it from https://learn.microsoft.com/azure/kinect-dk/sensor-sdk-download\n"
                "  or pass -DK4A_SDK_ROOT=\"C:/Program Files/Azure Kinect SDK vX.Y.Z/sdk\".\n"
                "  (looked for k4a/k4a.h and k4a.lib under the standard install dir)")
        endif()

        # Locate the runtime DLLs to co-locate with the exe. The depth engine
        # DLL is versioned (depthengine_2_0.dll today) so we glob it.
        get_filename_component(_TCAK_BIN_DIR "${K4A_LIBRARY}" DIRECTORY)
        get_filename_component(_TCAK_BIN_DIR "${_TCAK_BIN_DIR}/../bin" ABSOLUTE)
        find_file(K4A_RUNTIME_DLL NAMES k4a.dll HINTS "${_TCAK_BIN_DIR}" ${_TCAK_BIN_HINTS})
        file(GLOB _TCAK_DEPTHENGINE "${_TCAK_BIN_DIR}/depthengine_*.dll")
        if(K4A_RUNTIME_DLL)
            list(APPEND _TCAK_RUNTIME_DLLS "${K4A_RUNTIME_DLL}")
        endif()
        list(APPEND _TCAK_RUNTIME_DLLS ${_TCAK_DEPTHENGINE})

        add_library(k4a::k4a SHARED IMPORTED)
        set_target_properties(k4a::k4a PROPERTIES
            IMPORTED_IMPLIB "${K4A_LIBRARY}"
            IMPORTED_LOCATION "${K4A_RUNTIME_DLL}"
            INTERFACE_INCLUDE_DIRECTORIES "${K4A_INCLUDE_DIR}")
        message(STATUS "tcxAzureKinect: using k4a SDK headers ${K4A_INCLUDE_DIR}")
    else()
        # Linux without a CMake config: fall back to a plain header + lib search.
        find_path(K4A_INCLUDE_DIR NAMES k4a/k4a.h)
        find_library(K4A_LIBRARY  NAMES k4a)
        if(NOT K4A_INCLUDE_DIR OR NOT K4A_LIBRARY)
            message(FATAL_ERROR
                "tcxAzureKinect: the Azure Kinect Sensor SDK (libk4a) was not found.\n"
                "  Install libk4a-dev (see "
                "https://learn.microsoft.com/azure/kinect-dk/sensor-sdk-download).")
        endif()
        add_library(k4a::k4a UNKNOWN IMPORTED)
        set_target_properties(k4a::k4a PROPERTIES
            IMPORTED_LOCATION "${K4A_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${K4A_INCLUDE_DIR}")
    endif()
endif()

target_link_libraries(${ADDON_NAME} PUBLIC k4a::k4a)

# -----------------------------------------------------------------------------
# Windows: co-locate the k4a runtime DLLs with the app executable. Without
# depthengine_*.dll next to the exe, k4a_device_start_cameras() fails at runtime.
# tc_addon_bundle_file() is provided by use_addon.cmake; the app target copies
# the files post-build (see trussc_app.cmake).
# -----------------------------------------------------------------------------
if(WIN32 AND _TCAK_RUNTIME_DLLS AND COMMAND tc_addon_bundle_file)
    foreach(_dll ${_TCAK_RUNTIME_DLLS})
        tc_addon_bundle_file("${_dll}")
        message(STATUS "tcxAzureKinect: will bundle runtime DLL ${_dll}")
    endforeach()
endif()
