cmake_minimum_required(VERSION 4.0)

project(RSMotion)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")
endif()

# Add subdirectories
add_subdirectory(library)
add_subdirectory(example)
