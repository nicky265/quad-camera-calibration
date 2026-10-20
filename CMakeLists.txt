# CMakeLists.txt —— panostitch：四目鱼眼全景拼接器
#
# 标准构建流程（out-of-source build）：
#   cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
#   cmake --build build
#   ctest --test-dir build --output-on-failure
#
# 依赖：OpenCV 4（core/imgproc/imgcodecs，JSON 配置也由 OpenCV 解析）

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

# ---- 目标 ---------------------------------------------------------------
add_subdirectory(src)

if(PANOSTITCH_TOP_LEVEL AND BUILD_TESTING AND EXISTS "${PROJECT_SOURCE_DIR}/data/frames")
  add_subdirectory(tests)
endif()
