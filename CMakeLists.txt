cmake_minimum_required(VERSION 3.8)
# project(mjolnir)

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

file(GLOB_RECURSE mjolnir_SRCS "src/*.cpp" "src/*.cc")
file(GLOB_RECURSE mjolnir_HEADERS "include/*.h" "include/*.hpp")

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
        endif ()
        add_definitions(-DUSE_OPENCV)
    else ()
        set(USE_OPENCV OFF)
        message(STATUS "Can not found OpenCV, turned it off.")
    endif ()
else()
    message(STATUS "using simpleocv as opencv functionality!!")
    include_directories(3rd/simpleocv/include)
    file(GLOB_RECURSE OCV_SRCS "src/*.cpp" "src/*.cc" "src/*.hpp" "src/*.h")
endif()


if (BUILD_STATIC)
    add_library(mjolnir STATIC ${mjolnir_SRCS} ${mjolnir_HEADERS} ${OCV_SRCS})
else()
    add_library(mjolnir SHARED ${mjolnir_SRCS} ${mjolnir_HEADERS} ${OCV_SRCS})
endif()

target_link_libraries(mjolnir ${OpenCV_LIBS})
target_compile_options(mjolnir PUBLIC -fno-rtti)

set_target_properties(
    mjolnir
    PROPERTIES
    PUBLIC_HEADER "${mjolnir_HEADERS}")
target_include_directories(mjolnir PRIVATE .)

install(TARGETS mjolnir
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/mjolnir)
install(FILES include/mjolnir.pc
        DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/pkgconfig)
