cmake_minimum_required(VERSION 2.8.3)
project(manipulation2)

# First, set up the ROS Catkin package settings.  
# This will find a virtual package called 'catkin' that contains includes 
# and libraries aggregated from all the ROS catkin packages you depend on.
# 
# This macro creates:
# catkin_INCLUDE_DIRS, catkin_LIBRARIES, catkin_LIBRARY_DIRS
find_package(catkin REQUIRED)
 
# For system dependencies, use CMake's 'find_package' macros.
# These macros typically create (for a package named 'foo'):
# foo_INCLUDE_DIRS, foo_LIBRARIES, foo_LIBRARY_DIRS
find_package(Boost REQUIRED regex system)
 
# Set up the ROS Catkin package settings.
catkin_package()

# Some system dependencies don't have 'find_package' macros.  For these
# packages, CMake includes a helper function which can resolve the libraries
# using pkg-config, which most libraries support, and catkin builds generate.
include(FindPkgConfig)
pkg_check_modules(OpenRAVE REQUIRED openrave0.9)
 
# Add ALL the includes we need to build: stuff from catkin AND system dependencies.
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenRAVE_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

# CMake has add_executable and add_library functions to define build 'targets'.
add_library(${PROJECT_NAME}_plugin SHARED
    manipulationmain.cpp
    manipulation.cpp
    trajectoryproblem.cpp
)
target_link_libraries(${PROJECT_NAME}_plugin
    ${OpenRAVE_LIBRARIES} ${Boost_LIBRARIES})
set_target_properties(${PROJECT_NAME}_plugin PROPERTIES PREFIX "")
 
# At the end of the build, tell catkin to INSTALL your 'target'.
install(TARGETS ${PROJECT_NAME}_plugin
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/openrave0.9)
