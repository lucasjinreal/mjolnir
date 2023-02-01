cmake_minimum_required(VERSION 3.8)
project(mjolnir)

set(CMAKE_CXX_STANDARD 11)

option(BUILD_STATIC "enable to build static mjolnir lib" ON)
option(DEBUG "Enable debug mode, for develop" ON)
option(USE_OPENCV "using opencv or simpleocv? default false" OFF)

include(GNUInstallDirs)
include_directories(${PROJECT_SOURCE_DIR}/include/)
include_directories(include/mjolnir)

if (APPLE)
    # for mac M1 
    include_directories(/opt/homebrew/include/)
endif()

file(GLOB_RECURSE mjolnir_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc)
file(GLOB_RECURSE mjolnir_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h)

if(USE_OPENCV)
    message(STATUS "you are using opencv, this will disable simpleocv!!!")
    add_compile_definitions(USE_OPENCV)
    find_package(OpenCV 4.0 REQUIRED)
    if (OpenCV_FOUND)
        message(STATUS "Build with OpenCV")
        message(STATUS "    OpenCV version: " ${OpenCV_VERSION})
        message(STATUS "    OpenCV include path: " ${OpenCV_INCLUDE_DIRS})
        include_directories(${OpenCV_INCLUDE_DIRS})
        if (NOT ${OpenCV_VERSION} LESS "4.0.0")
            add_definitions(-DUSE_OPENCV4)
            add_definitions(-DUSE_OPENCV)
            message(STATUS "add micro on USE_OPENCV!!")
        endif ()
        add_definitions("-DUSE_OPENCV")
    else ()
        set(USE_OPENCV OFF)
        message(STATUS "Can not found OpenCV, turned it off.")
    endif ()
    set(mjolnir_HEADERS ${mjolnir_HEADERS} PARENT_SCOPE)
    set(mjolnir_SRCS ${mjolnir_SRCS} PARENT_SCOPE)
else()
    message(STATUS "using simpleocv as opencv functionality!!")
    include_directories(3rd/simpleocv/include)
    set(mjolnir_INCLUDE 3rd/simpleocv/include PARENT_SCOPE)
    file(GLOB_RECURSE OCV_SRCS "3rd/simpleocv/src/*.cpp" "3rd/simpleocv/src/*.cc" "3rd/simpleocv/src/*.hpp" "3rd/simpleocv/src/*.h")
    set(mjolnir_HEADERS ${mjolnir_HEADERS} ${PROJECT_SOURCE_DIR}/3rd/simpleocv/include/simpleocv.h PARENT_SCOPE)
    set(mjolnir_SRCS ${mjolnir_SRCS} ${OCV_SRCS} PARENT_SCOPE)
endif()


if (BUILD_STATIC)
    add_library(mjolnir STATIC ${mjolnir_SRCS} ${mjolnir_HEADERS})
else()
    add_library(mjolnir SHARED ${mjolnir_SRCS} ${mjolnir_HEADERS})
endif()

if (USE_OPENCV)
    target_link_libraries(mjolnir ${OpenCV_LIBS})
else()
    target_compile_options(mjolnir PUBLIC -fno-rtti)
endif()

set_target_properties(
    mjolnir
    PROPERTIES
    PUBLIC_HEADER "${mjolnir_HEADERS}")
target_include_directories(mjolnir PRIVATE .)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/libs/${CMAKE_SYSTEM_NAME})
set(CMAKE_INCLUDE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/include)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/libs/${CMAKE_SYSTEM_NAME})

install(TARGETS mjolnir
        ARCHIVE DESTINATION ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}
        LIBRARY DESTINATION ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}
        PUBLIC_HEADER DESTINATION ${CMAKE_INCLUDE_OUTPUT_DIRECTORY}/mjolnir)