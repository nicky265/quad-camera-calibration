# CMakeLists.txt —— panostitch（C++ 单张全景合成器）
#
# 依赖：OpenCV(core/imgproc/imgcodecs，本机 MinGW 静态安装) + nlohmann/json(third_party)
# 构建（MinGW + Ninja）：
#   cmake -S cpp -B cpp/build -G Ninja -DCMAKE_C_COMPILER=C:/sdk/mingw64/bin/gcc.exe \
#         -DCMAKE_CXX_COMPILER=C:/sdk/mingw64/bin/g++.exe
#   cmake --build cpp/build
cmake_minimum_required(VERSION 3.16)
project(panostitch CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# OpenCV 静态安装根目录（本机默认；可用 -DOPENCV_ROOT=... 覆盖）
# 说明：MinGW 静态安装的 OpenCVConfig.cmake 有缺陷，这里直接手链静态库。
set(OPENCV_ROOT "C:/sdk/opencv-mingw" CACHE PATH "OpenCV install root")
set(OPENCV_INCLUDE_DIR "${OPENCV_ROOT}/include")
set(OPENCV_LIB_DIR "${OPENCV_ROOT}/x64/mingw/staticlib")

add_executable(panostitch
  src/main.cpp
  src/config.cpp
  src/paths.cpp
  src/calib_io.cpp
  src/npy_io.cpp
  src/parallax.cpp
  src/blend.cpp
)

target_include_directories(panostitch PRIVATE include third_party "${OPENCV_INCLUDE_DIR}")

# 静态库顺序：被依赖者在前（imgcodecs → imgproc → core；第三方在最后）
target_link_libraries(panostitch PRIVATE
  "${OPENCV_LIB_DIR}/libopencv_imgcodecs4140.a"
  "${OPENCV_LIB_DIR}/libopencv_imgproc4140.a"
  "${OPENCV_LIB_DIR}/libopencv_core4140.a"
  "${OPENCV_LIB_DIR}/liblibjpeg-turbo.a"
  "${OPENCV_LIB_DIR}/liblibpng.a"
  "${OPENCV_LIB_DIR}/libzlib.a"
)

# Windows 系统库（OpenCV core 用到）
target_link_libraries(panostitch PRIVATE ws2_32)

# MinGW：静态链接运行库，生成不依赖额外 DLL 的可执行文件
if(MINGW)
  target_link_options(panostitch PRIVATE -static -static-libgcc -static-libstdc++)
endif()

set_target_properties(panostitch PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
