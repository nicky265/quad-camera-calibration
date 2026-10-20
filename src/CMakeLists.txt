# 核心拼接库：除 CLI 入口外的全部实现
add_library(panostitch_core STATIC
  blend.cpp
  calib_io.cpp
  config.cpp
  npy_io.cpp
  parallax.cpp
  paths.cpp
)
add_library(panostitch::core ALIAS panostitch_core)

target_compile_features(panostitch_core PUBLIC cxx_std_17)
set_target_properties(panostitch_core PROPERTIES CXX_EXTENSIONS OFF)

target_include_directories(panostitch_core
  PUBLIC
    "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
)

# cv::Mat 出现在公共头文件中，因此 OpenCV 是 PUBLIC 依赖
target_link_libraries(panostitch_core PUBLIC ${OpenCV_LIBS})
target_include_directories(panostitch_core PUBLIC ${OpenCV_INCLUDE_DIRS})

# nlohmann/json 只在 config.cpp 内部使用
if(PANOSTITCH_USE_SYSTEM_JSON)
  target_link_libraries(panostitch_core PRIVATE nlohmann_json::nlohmann_json)
else()
  target_include_directories(panostitch_core PRIVATE "${PROJECT_SOURCE_DIR}/third_party")
endif()

panostitch_enable_warnings(panostitch_core)

# ---- 命令行程序 ---------------------------------------------------------
add_executable(panostitch main.cpp)
target_compile_features(panostitch PRIVATE cxx_std_17)
set_target_properties(panostitch PROPERTIES CXX_EXTENSIONS OFF)
target_link_libraries(panostitch PRIVATE panostitch::core)

panostitch_enable_warnings(panostitch)

if(WIN32)
  target_link_libraries(panostitch PRIVATE ws2_32)
endif()

if(MINGW AND PANOSTITCH_STATIC_RUNTIME)
  target_link_options(panostitch PRIVATE -static -static-libgcc -static-libstdc++)
endif()

if(PANOSTITCH_TOP_LEVEL)
  install(TARGETS panostitch RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
endif()
