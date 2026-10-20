# CMakeLists.txt —— panostitch：四目鱼眼全景拼接器
#
# 标准构建流程（out-of-source build）：
#   cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
#   cmake --build build
#   ctest --test-dir build --output-on-failure
#
# 依赖：OpenCV 4（core/imgproc/imgcodecs）
#       nlohmann/json（默认使用 third_party 内置单头文件）

cmake_minimum_required(VERSION 3.16)

project(panostitch
  VERSION 0.1.0
  DESCRIPTION "Four-camera fisheye panorama stitcher"
  LANGUAGES CXX)

# ---- 选项 ---------------------------------------------------------------
option(PANOSTITCH_ENABLE_WARNINGS
  "Enable additional compiler warnings" ON)
option(PANOSTITCH_STATIC_RUNTIME
  "Statically link the GCC runtime when building with MinGW" ON)
option(PANOSTITCH_USE_SYSTEM_JSON
  "Use a system nlohmann_json package instead of the bundled header" OFF)

include(GNUInstallDirs)

if(CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
  set(PANOSTITCH_TOP_LEVEL ON)
else()
  set(PANOSTITCH_TOP_LEVEL OFF)
endif()

if(PANOSTITCH_TOP_LEVEL)
  include(CTest)
endif()

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
include(CompilerWarnings)

# ---- 依赖 ---------------------------------------------------------------
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs)

if(PANOSTITCH_USE_SYSTEM_JSON)
  find_package(nlohmann_json 3.11 REQUIRED)
endif()

# ---- 目标 ---------------------------------------------------------------
add_subdirectory(src)

if(PANOSTITCH_TOP_LEVEL AND BUILD_TESTING AND EXISTS "${PROJECT_SOURCE_DIR}/data/frames")
  add_subdirectory(tests)
endif()
