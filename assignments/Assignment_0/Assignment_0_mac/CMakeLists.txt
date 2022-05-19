cmake_minimum_required (VERSION 2.8)
cmake_policy(VERSION 2.8)

project (Assignment_0)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules/")

list(APPEND CMAKE_CXX_FLAGS "-std=c++11")

find_package(GLFW3 REQUIRED)
message(STATUS "Found GLFW3 in ${GLFW3_INCLUDE_DIR}")

SET(LIBS ${GLFW3_LIBRARY})

add_library(GLAD "src/glad.c")
set(LIBS ${LIBS} GLAD)
include_directories(${CMAKE_SOURCE_DIR}/includes)

add_executable(Assignment_0 src/main.cpp)
target_link_libraries(Assignment_0 ${LIBS})
target_link_libraries(Assignment_0 ${GLFW3_LIBRARY})