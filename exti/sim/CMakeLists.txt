######################################################################
# @author      : karl (karlsasssie@gmail.com)
# @file        : CMakeLists
# @created     : Friday Apr 07, 2023 15:39:05 CEST
######################################################################

cmake_minimum_required(VERSION 2.8)
project(sim CXX)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(${PROJECT_NAME} 
  main.cpp
  linear_encoder.cpp
  state_estimation.cpp
  ekf.cpp
  accelerometer.cpp
)

