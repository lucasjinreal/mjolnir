cmake_minimum_required(VERSION 3.5)
project(mjolnir VERSION 1.3.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS  "-fPIC")

option(BUILD_STATIC "enable to build static mjolnir lib" OFF)
option(DEBUG "Enable debug mode, for develop" ON)

include(GNUInstallDirs)
include_directories(${PROJECT_SOURCE_DIR}/include/)

if (APPLE)
    # for mac M1 
    include_directories(/opt/homebrew/include/)
endif()

find_package(OpenCV 4.0 REQUIRED)
if (OpenCV_FOUND)
    message(STATUS "Build with OpenCV")
    message(STATUS "    OpenCV version: " ${OpenCV_VERSION})
    message(STATUS "    OpenCV include path: " ${OpenCV_INCLUDE_DIRS})
    include_directories(${OpenCV_INCLUDE_DIRS})
    if (NOT ${OpenCV_VERSION} LESS "4.0.0")
        add_definitions(-DUSE_OPENCV4)
    endif ()
    add_definitions(-DUSE_OPENCV)
else ()
    set(USE_OPENCV OFF)
    message(STATUS "Can not found OpenCV, turned it off.")
endif ()

file(GLOB_RECURSE mjolnir_SRCS "src/*.cpp" "src/*.cc")
file(GLOB_RECURSE mjolnir_HEADERS "include/*.h" "include/*.hpp")

add_library(mjolnir SHARED ${mjolnir_SRCS} ${mjolnir_HEADERS})
if (BUILD_STATIC)
    add_library(mjolnir_static STATIC ${mjolnir_SRCS} ${mjolnir_HEADERS})
endif()

target_link_libraries(mjolnir ${OpenCV_LIBS})

set_target_properties(
        mjolnir
        PROPERTIES
        PUBLIC_HEADER "${mjolnir_HEADERS}")
target_include_directories(mjolnir PRIVATE .)

if (BUILD_STATIC) 
set_target_properties(
        mjolnir_static
        PROPERTIES
        PUBLIC_HEADER "${mjolnir_HEADERS}")
        target_include_directories(mjolnir_static PRIVATE .)
endif()

install(TARGETS mjolnir
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/mjolnir)
if (BUILD_STATIC)
install(TARGETS mjolnir_static
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/mjolnir)
endif()
install(FILES include/mjolnir.pc
        DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/pkgconfig)
