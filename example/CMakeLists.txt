add_executable(app src/main.cpp)

targeT_link_libraries(app PRIVATE rsmotion)

set_target_properties(app PROPERTIES CXX_STANDARD 20 CXX_STANDARD_REQUIRED ON)

target_link_options(
  app
  PRIVATE
  "SHELL:-sENVIRONMENT='web'"
  -sEXPORT_ES6
  -sEXPORT_NAME=CppApp
  -sMIN_WEBGL_VERSION=2
  -sMAX_WEBGL_VERSION=2
  -sALLOW_MEMORY_GROWTH=1
  -sINVOKE_RUN=0
  # RelWithDebInfo build flags, enable separate dwarf
  # to reduce wasm file size
  $<$<CONFIG:RELWITHDEBINFO>:-gseparate-dwarf=${CMAKE_CURRENT_BINARY_DIR}/app.dwarf>
  $<$<CONFIG:RELWITHDEBINFO>:-g>
  $<$<CONFIG:RELWITHDEBINFO>:-O2>
  # Debug build flags
  $<$<CONFIG:DEBUG>:-fwasm-exceptions>
  $<$<CONFIG:DEBUG>:-g>
  $<$<CONFIG:DEBUG>:-O0>
  # Exported Emscripten runtime methods
  "SHELL:-sEXPORTED_RUNTIME_METHODS='[\"callMain\", \"ccall\", \"cwrap\"]'"
  # Export our C++ functions
  "SHELL:-sEXPORTED_FUNCTIONS='[\"_main\", \"_getCarPositionX\", \"_getCarPositionY\", \"_getCarPositionZ\", \"_getCarOrientationX\", \"_getCarOrientationY\", \"_getCarOrientationZ\", \"_getPathPointCount\", \"_getPathPointX\", \"_getPathPointY\", \"_getPathPointZ\", \"_getFinishPositionX\", \"_getFinishPositionY\", \"_getFinishPositionZ\", \"_getFinishOrientationX\", \"_getFinishOrientationY\", \"_getFinishOrientationZ\"]'")

# Custom command and target to copy our compiled WASM and JS
# files from the C++ build directory into the web app's source
# directory under web/src/cpp

set(WEB_OUT_DIR ${PROJECT_SOURCE_DIR}/web/src/cpp)

add_custom_command(
  DEPENDS app
  OUTPUT ${WEB_OUT_DIR}/app.js ${WEB_OUT_DIR}/app.wasm
  COMMAND cmake -E make_directory ${WEB_OUT_DIR}
  COMMAND cmake -E copy_if_different ${CMAKE_CURRENT_BINARY_DIR}/app.js
          ${CMAKE_CURRENT_BINARY_DIR}/app.wasm ${WEB_OUT_DIR})

add_custom_target(
  copy_wasm_to_app ALL
  DEPENDS ${WEB_OUT_DIR}/app.js ${WEB_OUT_DIR}/app.wasm
  COMMENT "Copying wasm build to ${WEB_OUT_DIR}")