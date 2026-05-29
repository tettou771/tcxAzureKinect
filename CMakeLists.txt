# =============================================================================
# tcxAzureKinect - Azure Kinect DK backend for tcxDepthCamera
# =============================================================================
#
# Implements the tcxDepthCamera interface on top of the Azure Kinect Sensor SDK
# (k4a). The k4a SDK is officially supported on Windows and Linux only; this
# addon is not buildable on macOS.
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

target_include_directories(${ADDON_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    # tcxDepthCamera is a header-only interface addon sitting next to us.
    ${CMAKE_CURRENT_SOURCE_DIR}/../tcxDepthCamera/src
)

target_link_libraries(${ADDON_NAME} PUBLIC TrussC)

# Azure Kinect Sensor SDK. Provides the imported target k4a::k4a.
find_package(k4a REQUIRED)
target_link_libraries(${ADDON_NAME} PUBLIC k4a::k4a)
