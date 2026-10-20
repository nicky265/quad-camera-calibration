// blend.hpp —— 融合：简单加权 / 多频段 / 找缝
//
// 对应 Python: common/blend.py。
//   featherBlend      ~ feather_blend
//   blendMultiband    ~ blend_multiband
//   computeSeamWeights~ seam_weights
//
// 数据类型：warps = CV_32FC3 (H×W)；weights/valids = CV_32F/CV_8U (H×W)。
#pragma once

#include <vector>
#include <opencv2/core.hpp>

namespace pan {

// clip 到 [0,255] 后**向零截断**转 uint8（与 numpy clip().astype(uint8) 一致）。
cv::Mat truncToU8(const cv::Mat& src);

// 简单加权平均：pano = Σ warp_i * w_i / Σ w_i（w 已逐像素归一化也可）。
cv::Mat featherBlend(const std::vector<cv::Mat>& warps,
                     const std::vector<cv::Mat>& weights);

// 多频段（Laplacian 金字塔）融合。levels 为层数；threads 为并行线程数（1=串行）。
cv::Mat blendMultiband(const std::vector<cv::Mat>& warps,
                       const std::vector<cv::Mat>& weights, int levels, int threads = 1);

// 找缝（动态规划）生成每相机权重：重叠带内按最小失配路径归属单相机，
// 再羽化并逐像素归一化。返回 n 张 CV_32F (H×W)。
std::vector<cv::Mat> computeSeamWeights(const std::vector<cv::Mat>& warps,
                                        const std::vector<cv::Mat>& valids,
                                        const std::vector<double>& centerAngles,
                                        double fov_h_deg,
                                        double feather_px = 10.0,
                                        double band_frac = 0.6,
                                        double lam = 1.5,
                                        int threads = 1);

} // namespace pan
