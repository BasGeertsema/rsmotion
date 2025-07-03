# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

RSMotion is a C++11 library for calculating optimal paths for car movements using the Reeds-Shepp Cars algorithm. The project includes:
- Core C++ library (minimal dependencies). (FINISHED, DO NOT MODIFY)
- A C++ example application using the library, compiled to WebAssembly (Wasm)
- A typeScript web application visualizing the C++ example application

The rsmotion library is FINISHED. There are NO modificiations to the library in the /library directory!

## Build Commands

### C++ Library and Example (Using CMake with Emscripten)

```bash
# From build/ directory
cmake --build .

# This will:
# 1. Build the static library (librsmotion.a)
# 2. Build the WebAssembly example app
# 3. Copy app.js and app.wasm to web/src/cpp/
```

### Web Application

```bash
# From web/ directory
npm install
npm run build    # Build for production
npm run serve    # Development server with hot reload
npm run deploy   # Production build
```

### Development Workflow

To make changes and see them in the web app:
1. Edit C++ code
2. Run `cmake --build .` in build/ directory (rebuilds and copies WASM)
3. Web dev server will auto-reload with new WASM

## Architecture

### Core Library Structure
- `library/src/rsmotion.cpp` - Single implementation file
- `library/include/rsmotion/` - Public headers:
  - `rsmotion.h` - Main API (CarState, Path, SearchShortestPath)
  - `math.h` - Vector/quaternion mathematics
  - `coordinatesystem.h` - Coordinate system configuration (default: RIGHT, UP, FORWARD)

## Example application

The example application is a combination of a C++ component compiled to WebAssembly (WASM) and the webapplication in typescript. The example application demonstrates a single car that has to 'drive' towards a random point on an optimal path. Once the car reaches it's destination a new random destination point will be determined, along with a new optimal path. The C++ component (compiled down to webassembly) is responsible for the car simulation: generating destinations, paths and moving the car along the path. The typescript web application is responsible for the visualization using WebGL.

### Example C++ component compiled as WASM module
Simulates the car driving and generates new destinations and optimal paths. Runs a main loop that continously 'drives' the car and updates the state.

- `example/src/main.cpp`
- Compiled to WebAssembly with Emscripten
- Shows real-time path generation with mouse interaction

### Web Application
Visualizes the car movements. The scene is as follows:
* The 'ground' on which the car drives is a simple checkered plane with width = 20 and length = 20
* The car are two simple boxes stacked on top of each other. The top box is the cabin and is slightly smaller and toward the rear of the car.
* The calculated optimal path that the car takes is always shown by line segments

The current STATE of the car and the path are available in the WASM component and must be queries by the typescript application.

It is visualized through a orthographic projection, looking at an angle from above onto the ground (like a bird perspective).

- TypeScript + WebPack
- Loads and runs the WASM module
- WebGL rendering of paths and car movement

## Code Style Guidelines
- C++14 for library (though API is C++11 compatible)
- C++20 for example app
- Compiler flags: `-Wall -Wextra -pedantic -fpermissive`
- Single source file design for easy integration
- No external dependencies in core library

## Important Notes
- No formal test suite - example app serves as integration test
- Turn radius fixed to unit circle
- The *COORDINATE SYSTEM* is that for a 3d vector (x, y, z) : x = left/right/width, y = up/down/height and z = forward/backwards/length .