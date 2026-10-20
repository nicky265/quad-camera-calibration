// npy_io.hpp —— 极简 .npy 读取（仅 '<f4'，非 fortran）
//
// 用于 gain.npy（shape (3,)）与可选的 vignette.npy（float32 图）。
// 与 Python: np.save/io_calib.save_gain 产出的文件格式对应。
#pragma once

#include <string>
#include <opencv2/core.hpp>

namespace pan {

// 读取 .npy（float32）。一维返回 N×1，二维返回 rows×cols。
// 不支持 fortran_order；descr 必须是 '<f4'/'<f4' 系列。
cv::Mat loadNpy(const std::string& path);

} // namespace pan
