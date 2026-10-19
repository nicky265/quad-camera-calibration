# panostitch（C++ 单张全景合成器）

C++ 版本的**单张高质量合成**程序，功能对齐 Python `tools/stitch_preview.py`，
用于①验证算法一致②测量 C++ 开销。本目录已包含运行所需的标定产物和示例帧，
可独立构建、运行。

> 本版只做**单张合成**（读磁盘帧），不做实时采集。默认单线程；`--threads N` 开启多线程。
>
> **终端输出为英文/ASCII**（避免 Windows 控制台按 GBK 解码导致的乱码）；**文档与代码注释仍为中文**。

## 1. 依赖

| 依赖 | 版本 | 来源 |
|---|---|---|
| OpenCV（core/imgproc/imgcodecs，静态） | 4.14.0 | 由 OpenCV 源码用 MinGW 编译，安装到 `C:/sdk/opencv-mingw` |
| nlohmann/json | v3.12.0 | 单头文件，已放 `cpp/third_party/nlohmann/json.hpp` |
| 工具链 | MinGW-w64 g++ 15.2 + Ninja + CMake≥3.16 | 本机 `C:\sdk\mingw64` |

> 说明：pip 的 `opencv-python` **不含 C++ 头/库**；本项目的 C++ OpenCV 是单独从源码
> 编译的**最小静态库**（core/imgproc/imgcodecs，含自带 libjpeg-turbo/libpng/zlib）。
> 运行 `panostitch.exe` 不需要额外的 OpenCV DLL。

## 2. 目录结构

```
cpp/
├─ config.json                 # C++ 配置（路径相对于本文件所在目录）
├─ data/
│  ├─ calib/
│  │  ├─ calib_meta.json       # 标定元信息（仅记录，程序不读取）
│  │  └─ camera_0..3/          # 每路相机的标定产物
│  │     ├─ intrinsics.yml
│  │     ├─ extrinsics.yml
│  │     ├─ gain.npy
│  │     ├─ lut.bin
│  │     └─ weight.bin
│  └─ frames/
│     └─ 0.bmp 1.bmp 2.bmp 3.bmp
├─ output/                     # 运行结果（自动创建）
├─ include/
│  ├─ config.hpp
│  ├─ paths.hpp
│  ├─ calib_io.hpp
│  ├─ npy_io.hpp
│  ├─ parallax.hpp
│  ├─ blend.hpp
│  └─ parallel.hpp
├─ src/
│  ├─ main.cpp
│  ├─ config.cpp
│  ├─ paths.cpp
│  ├─ calib_io.cpp
│  ├─ npy_io.cpp
│  ├─ parallax.cpp
│  └─ blend.cpp
├─ third_party/nlohmann/json.hpp
├─ CMakeLists.txt
└─ README.md
```

### 与 Python 的对应关系

| C++ | Python |
|---|---|
| `config.*` | `common/config.py` |
| `calib_io.*` | `common/io_calib.py`（LUT/权重/内外参） |
| `npy_io.*` | `io_calib.load_gain`（`.npy` 解析） |
| `parallax.*` | `pipeline/gen_lut.py` + `common/fisheye.py` |
| `blend.*` | `common/blend.py` |
| `main.cpp` | `tools/stitch_preview.py` |

## 3. 构建

```powershell
# 配置（MinGW + Ninja；OpenCV 路径默认 C:/sdk/opencv-mingw，可 -DOPENCV_ROOT=... 覆盖）
cmake -S cpp -B cpp/build -G Ninja `
      -DCMAKE_C_COMPILER=C:/sdk/mingw64/bin/gcc.exe `
      -DCMAKE_CXX_COMPILER=C:/sdk/mingw64/bin/g++.exe
cmake --build cpp/build
# 产物：cpp/build/panostitch.exe（静态链接运行库，独立可执行）
```

## 4. 路径规则

- 默认配置为 `cpp/config.json`：程序从 exe 所在目录向上查找 `config.json`，
  因此**从任意工作目录启动都能找到 `cpp/` 下的依赖**。
- `config.json` 里的 `paths.calib_root`、`paths.frames_root`、`paths.out_root`
  均为**相对于 config.json 所在目录**的路径；写成绝对路径则原样使用。
- 显式传入的 `--config`、`--frames`、`--out` 按**当前工作目录**解析。
- 默认输出目录不存在时会自动创建。

## 5. 用法

```powershell
# 默认：读 cpp/data/calib 的 LUT，多频段 + 找缝，输出 cpp/output/cpp_pano.jpg
.\cpp\build\panostitch.exe --blend multiband --seam

# 视差扫参（内存构造映射，不改 lut.bin）
.\cpp\build\panostitch.exe --parallax inmem --ground-height 0.5 --blend multiband --seam
.\cpp\build\panostitch.exe --parallax inmem --focus-distance 4.5 --blend multiband

# 多线程
.\cpp\build\panostitch.exe --threads 4 --blend multiband --seam

# 指定其它输入/输出（相对当前工作目录）
.\cpp\build\panostitch.exe --frames data\preview --out output\test.jpg
```

| 参数 | 默认 | 说明 |
|---|---|---|
| `--frames DIR` | `cpp/data/frames` | 4 路源帧目录（命名 `0.*` / `cam0.*` / `00.*`） |
| `--blend {feather,multiband}` | `multiband` | 融合方式 |
| `--seam` | 关 | 先找缝再融合 |
| `--levels N` | 5 | 多频段层数 |
| `--parallax {lut,inmem}` | `lut` | lut=读已烘焙 LUT；inmem=内存现算 |
| `--ground-height H` | 无 | inmem 地面补偿（米） |
| `--focus-distance D` | 无 | inmem 单距离补偿（米） |
| `--far-distance F` / `--max-ground-distance M` | 0 / 30 | inmem 地面模式参数 |
| `--threads N` | 1 | 线程数；`0`=取硬件线程数 |
| `--out PATH` | `cpp/output/cpp_pano.jpg` | 输出图 |
| `--config PATH` | `cpp/config.json` | 配置 |

调试用（可删）：`--dump-warp i file.bin`（导出第 i 路 remap 结果，float32）、
`--dump-pano file.bin`（导出融合后的 float 全景）。

## 6. 标定数据更新

重新运行 Python 标定后，把新产物同步到 `cpp/data/`：

```powershell
# 标定产物：output/camera_0..3 -> cpp/data/calib
0..3 | ForEach-Object {
  Copy-Item "output\camera_$_" "cpp\data\calib\" -Recurse -Force
}
Copy-Item output\calib_meta.json cpp\data\calib\ -Force

# 示例帧：data/preview/0..3.bmp -> cpp/data/frames
0..3 | ForEach-Object {
  Copy-Item "data\preview\$_.bmp" "cpp\data\frames\" -Force
}
```

## 7. 验证（与 Python 逐像素一致）

同参数下与 `tools/stitch_preview.py` 输出**逐像素完全相同**（`max diff = 0`）：

| 模式 | 结果 |
|---|---|
| `--blend feather`（读 LUT） | 一致 |
| `--blend multiband --seam`（读 LUT） | 一致 |
| `--parallax inmem --ground-height 0.5 --seam` | 一致 |
| `--parallax inmem --focus-distance 4.5` | 一致 |

关键点：numpy `clip().astype(uint8)` 是**向零截断**（floor），OpenCV `convertTo` 是四舍五入；
C++ 用 `truncToU8()`（clip 后 `(int)` 截断）与之对齐，见 `blend.cpp`。

## 8. 性能（2048×512，单线程，本机参考，取 3 次最小）

| 模式 | C++ | Python | 加速 |
|---|---|---|---|
| multiband+seam（读 LUT） | ~1.2 s | ~2.5 s | ~2.1× |
| ground multiband+seam | ~1.1 s | ~2.6 s | ~2.3× |
| focus multiband（无 seam） | ~0.53 s | ~0.61 s | ~1.2× |

- 主要收益来自**找缝 DP**（C++ ~0.53 s vs Python ~1.9 s）。
- remap/多频段调用的是同一套 OpenCV，二者接近。
- 单线程；实时优化（多线程/ROI/`--opt16`/GPU）留待后续。

## 9. 多线程（`--threads N`）

`--threads 1`（默认）为单线程版本；`--threads N>1`（或 `0`=硬件线程数）为**多线程版本**，
两者输出**逐位一致**（并行只作用于彼此独立、写不同输出的任务，不改变浮点累加顺序，
见 `include/parallel.hpp`）。

并行位置：
- **光度+remap**：4 台相机并行；
- **视差映射**（inmem）：4 台相机并行；
- **找缝**：灰度图按相机并行、4 条接缝对并行（各自覆盖的输出列互不重叠）、权重组装按相机并行；
- **多频段**：金字塔按相机并行、各层融合并行。

> OpenCV 内部仍固定单线程（`cv::setNumThreads(1)`），并行度完全由本程序控制，保证可复现。

实测（2048×512，multiband+seam，本机）：

| 线程 | 合成合计 | 光度+remap | 找缝 | 融合 |
|---|---|---|---|---|
| 1 | ~1050 ms | ~258 | ~526 | ~266 |
| 4 | ~483 ms | ~108 | ~181 | ~194 |
| 8 | ~462 ms | — | ~172 | — |

→ 4 线程约 **2.2×**（找缝约 2.9×）；超过核心数后趋于饱和。

## 10. 坐标约定（与全工程一致）

- 世界系 = 相机 0 系，沿用 OpenCV 相机约定（+X 右、+Y 下、+Z 前），**地面在 Y=+H**。
- `X_world = R_cw · X_cam + T_cw`，相机光心 `P = T_cw`。
- 鱼眼等距：`theta=atan2(r,z)`, `theta_d=theta·(1+k1 t²+…)`，含 θ_fold/图像圆/边界判定。
- 柱面：`f=W/rad(fov)`，`theta=(u+0.5−W/2)/f`，方向 `(sinθ, h/f, cosθ)` 归一化。

## 11. 发布说明

- `data/` 下包含约 69MB 的 LUT、权重和示例帧，直接提交 Git 即可运行。
- `build/` 和 `output/` 为生成物，不纳入版本库。
- 若后续仓库体积过大，可再迁移到 Git LFS 或 GitHub Release 附件。
