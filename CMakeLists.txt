cmake_minimum_required(VERSION 3.0.0)
project(HelloGTSAM VERSION 0.1.0)

include(CTest)
enable_testing()

add_executable(HelloGTSAM main.cpp)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

## Boost
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost 1.58 REQUIRED COMPONENTS filesystem)
message(STATUS "Boost version: ${Boost_VERSION}")

target_link_libraries(HelloGTSAM LINK_PUBLIC ${Boost_LIBRARIES})

