cmake_minimum_required(VERSION 3.0.0)
project(HandEyeCpp VERSION 0.1.0)

include(CTest)
enable_testing()

set(default_build_type "Release")

##############################################################################
# GTSAM
# # Include GTSAM CMake tools
# find_package(GTSAMCMakeTools)
# include(GtsamBuildTypes)
# include(GtsamTesting)
# include(GtsamMatlabWrap)

# # Ensure that local folder is searched before library folders
# include_directories(BEFORE "${PROJECT_SOURCE_DIR}")

# Find GTSAM components
find_package(GTSAM REQUIRED)

find_package(yaml-cpp REQUIRED)

##############################################################################
add_executable(SimulatedCalibration SimulatedCalibration.cpp PoseSimulation.cpp)
target_link_libraries(SimulatedCalibration PUBLIC gtsam)

add_executable(PoseAveraging PoseAveraging.cpp PoseSimulation.cpp)
target_link_libraries(PoseAveraging PUBLIC gtsam)

add_executable(Jacobians Jacobians.cpp PoseSimulation.cpp)
target_link_libraries(Jacobians PUBLIC gtsam)

add_executable(HelloYaml HelloYaml.cpp)
target_link_libraries(HelloYaml PUBLIC yaml-cpp)

##############################################################################
set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

