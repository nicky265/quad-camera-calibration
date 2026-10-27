// npy_io.hpp —— 极简 .npy 读取（仅小端 '<f4'，非 fortran）
//
// 用于 gain.npy（shape (3,)）等标定产物。
// 与 Python: np.save/io_calib.save_gain 产出的文件格式对应。
#pragma once

#include <string>

#include <opencv2/core.hpp>

namespace pan {

// 读取 .npy（float32）。一维返回 N×1，二维返回 rows×cols。
// 不支持 fortran_order、大端字节序和 3 维及以上数组。
cv::Mat loadNpy(const std::string& path);

} // namespace pan
