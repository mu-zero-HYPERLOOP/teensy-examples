######################################################################
# @author      : karl (karlsasssie@gmail.com)
# @file        : CMakeLists
# @created     : Friday Apr 07, 2023 15:39:05 CEST
######################################################################

cmake_minimum_required(VERSION 2.8)
project(moving-average-test CXX)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(moving-average src/main.cpp)

