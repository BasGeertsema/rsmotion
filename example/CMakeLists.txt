cmake_minimum_required (VERSION 2.6)

set (CMAKE_CXX_STANDARD 11)

# default to release build
SET(CMAKE_BUILD_TYPE Release)

# add rsmotion library
include_directories(../library/include)
add_library(rsmotion ../library/src/rsmotion.cpp)

# add example executable
project (example)

include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/include/compat/msvc)

add_definitions(-D_ITERATOR_DEBUG_LEVEL=0)

# this example only supports/embeds win64 libraries currently
link_directories(${CMAKE_SOURCE_DIR}/lib/win64)

file (COPY shaders/fs_cubes.bin shaders/vs_cubes.bin DESTINATION ./)

add_executable(example src/main.cpp src/bounds.cpp src/debugdraw/debugdraw.cpp)

target_link_libraries(example 
    rsmotion
    debug bimgDebug
    debug bxDebug
    debug bgfxDebug
    optimized bimgRelease
    optimized bxRelease
    optimized bgfxRelease
    general glfw3)