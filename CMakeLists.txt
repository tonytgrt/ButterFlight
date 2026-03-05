cmake_minimum_required(VERSION 3.25)
project(ButterFlight)

# ---- Maya installation path -----------------------------------------------
set(MAYA_LOCATION "C:/Program Files/Autodesk/Maya2026")

if(NOT EXISTS "${MAYA_LOCATION}")
    message(FATAL_ERROR
        "Maya not found at \"${MAYA_LOCATION}\". "
        "Set MAYA_LOCATION to your Maya 2026 installation directory.")
endif()

# ---- C++ standard -------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- CMake modules ------------------------------------------------------------
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

# ---- Output directories -------------------------------------------------------
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# ---- Find Maya ----------------------------------------------------------------
find_package(Maya REQUIRED)

# ---- Source files -------------------------------------------------------------
set(SOURCES
    src/PluginMain.cpp
    src/BFSimulateCmd.cpp
    src/BFWingModel.cpp
    src/BFAerodynamics.cpp
    src/BFCurlNoise.cpp
    src/BFManeuverController.cpp
)

set(HEADERS
    src/BFState.h
    src/BFSimulateCmd.h
    src/BFWingModel.h
    src/BFAerodynamics.h
    src/BFCurlNoise.h
    src/BFManeuverController.h
)

# ---- Plugin target ------------------------------------------------------------
add_library(butterFlight SHARED ${SOURCES} ${HEADERS})

target_link_libraries(butterFlight PRIVATE Maya::Maya)

target_include_directories(butterFlight
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src"
    PRIVATE Maya::Maya
)

MAYA_PLUGIN(butterFlight)

# ---- Install rules ------------------------------------------------------------
install(TARGETS butterFlight ${MAYA_TARGET_TYPE}
    DESTINATION plug-ins/${MAYA_VERSION})

install(FILES src/mel/butterFlight_ui.mel
    DESTINATION scripts)
