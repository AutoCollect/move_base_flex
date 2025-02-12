cmake_minimum_required(VERSION 3.10)

# Suppress developer warnings (equivalent to using -Wno-dev on the command line)
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "Suppress developer warnings")

# Set policies to avoid deprecation warnings
cmake_policy(SET CMP0148 OLD)
cmake_policy(SET CMP0167 OLD)

project(mbf_costmap_core)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3")

find_package(catkin REQUIRED
        COMPONENTS
            std_msgs
            geometry_msgs
            mbf_abstract_core
            mbf_utility
            tf
            costmap_2d
            nav_core
        )

catkin_package(
    INCLUDE_DIRS
        include
    CATKIN_DEPENDS
            std_msgs
            geometry_msgs
            mbf_abstract_core
            mbf_utility
            tf
            costmap_2d
            nav_core
)

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)
