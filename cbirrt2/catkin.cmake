cmake_minimum_required(VERSION 2.8.3)
project(cbirrt2)

# First, set up the ROS Catkin package settings.  
# This will find a virtual package called 'catkin' that contains includes 
# and libraries aggregated from all the ROS catkin packages you depend on.
# 
# This macro creates:
# catkin_INCLUDE_DIRS, catkin_LIBRARIES, catkin_LIBRARY_DIRS
find_package(catkin REQUIRED COMPONENTS openrave_catkin)
 
# For system dependencies, use CMake's 'find_package' macros.
# These macros typically create (for a package named 'foo'):
# foo_INCLUDE_DIRS, foo_LIBRARIES, foo_LIBRARY_DIRS
find_package(Boost REQUIRED regex system)
find_package(OpenRAVE REQUIRED)
 
# Set up the ROS Catkin package settings.
catkin_package()
catkin_python_setup()

# Some system dependencies don't have 'find_package' macros.  For these
# packages, CMake includes a helper function which can resolve the libraries
# using pkg-config, which most libraries support, and catkin builds generate.
 
# Add ALL the includes we need to build: stuff from catkin AND system dependencies.
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${OpenRAVE_INCLUDE_DIRS}
)

# CMake has add_executable and add_library functions to define build 'targets'.
openrave_plugin("${PROJECT_NAME}_plugin"
    cbirrtmain.cpp
    TaskSpaceRegion.cpp
    cbirrt.cpp
    cbirrtproblem.cpp
)
target_link_libraries("${PROJECT_NAME}_plugin"
    qhull
    newmat
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)

catkin_add_gtest(test_wam7_unconstrained test/test_wam7_unconstrained.cpp)
target_link_libraries(test_wam7_unconstrained
  ${Boost_LIBRARIES}
  ${OpenRAVE_LIBRARIES}
  ${OpenRAVE_CORE_LIBRARIES}
)

catkin_add_gtest(test_wam7_constrained test/test_wam7_constrained.cpp TaskSpaceRegion.cpp)
target_link_libraries(test_wam7_constrained
  ${Boost_LIBRARIES}
  ${OpenRAVE_LIBRARIES}
  ${OpenRAVE_CORE_LIBRARIES}
)

catkin_add_gtest(test_params test/test_params.cpp TaskSpaceRegion.cpp)
target_link_libraries(test_params
  ${Boost_LIBRARIES}
  ${OpenRAVE_LIBRARIES}
  ${OpenRAVE_CORE_LIBRARIES}
)
