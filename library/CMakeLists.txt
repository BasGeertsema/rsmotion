cmake_minimum_required(VERSION 4.0)

project(RSMotionLibrary)

# Create static library
add_library(rsmotion STATIC
    src/rsmotion.cpp
)

# Set include directories for the library
target_include_directories(rsmotion PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Set C++ standard for the library
target_compile_features(rsmotion PUBLIC cxx_std_14)

# Add permissive flag to handle template function dependencies
target_compile_options(rsmotion PRIVATE -fpermissive)
