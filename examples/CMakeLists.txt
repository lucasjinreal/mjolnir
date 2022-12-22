cmake_minimum_required(VERSION 3.8)
project(examples)

set(CMAKE_CXX_STANDARD 11)

include_directories(../build/include)
link_directories(../build/libs/${CMAKE_SYSTEM_NAME})

add_executable(demo_det demo_det.cc)
target_link_libraries(demo_det mjolnir)