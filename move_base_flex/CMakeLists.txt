cmake_minimum_required(VERSION 3.10)

# Suppress developer warnings (equivalent to using -Wno-dev on the command line)
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "Suppress developer warnings")

# Set policies to avoid deprecation warnings
cmake_policy(SET CMP0148 OLD)
cmake_policy(SET CMP0167 OLD)

project(move_base_flex)
find_package(catkin REQUIRED)
catkin_metapackage()
