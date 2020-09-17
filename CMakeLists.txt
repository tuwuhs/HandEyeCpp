cmake_minimum_required(VERSION 3.0.0)
project(HelloGTSAM VERSION 0.1.0)

include(CTest)
enable_testing()

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


##############################################################################
add_executable(HelloGTSAM main.cpp PoseSimulation.cpp)

target_link_libraries(HelloGTSAM PUBLIC gtsam)


##############################################################################
set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

