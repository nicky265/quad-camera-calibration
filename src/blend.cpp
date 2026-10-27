// blend.cpp —— 融合实现
#include <panostitch/blend.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>

#include <panostitch/parallel.hpp>

namespace pan {

namespace {

constexpr double kPi = 3.14159265358979323846;

double wrapAngleDeg(double angle) {
    double r = std::fmod(angle + 180.0, 360.0);
    if (r < 0) r += 360.0;
    return r - 180.0;
}

std::vector<cv::Mat> laplacianPyramid(const cv::Mat& img, int levels) {
    if (img.empty()) throw std::runtime_error("laplacianPyramid: empty image");
    if (levels < 1) throw std::runtime_error("laplacianPyramid: levels must be >= 1");
    const int max_levels =
        1 +
        static_cast<int>(std::floor(std::log2(static_cast<double>(std::min(img.rows, img.cols)))));
    if (levels > max_levels) {
        throw std::runtime_error("laplacianPyramid: too many levels for image size");
    }

    std::vector<cv::Mat> gaussian;
    gaussian.reserve(levels);
    gaussian.push_back(img.clone());
    for (int i = 1; i < levels; ++i) {
        cv::Mat down;
        cv::pyrDown(gaussian[i - 1], down);
        gaussian.push_back(down);
    }

    std::vector<cv::Mat> laplacian(levels);
    for (int i = 0; i < levels - 1; ++i) {
        cv::Mat up;
        cv::pyrUp(gaussian[i + 1], up, gaussian[i].size());
        laplacian[i] = gaussian[i] - up;
    }
    laplacian[levels - 1] = gaussian[levels - 1];
    return laplacian;
}

cv::Mat grayOf(const cv::Mat& warp) {
    const cv::Mat u8 = truncToU8(warp);
    cv::Mat gray, gray_f;
    cv::cvtColor(u8, gray, cv::COLOR_BGR2GRAY);
    gray.convertTo(gray_f, CV_32F);
    cv::GaussianBlur(gray_f, gray_f, cv::Size(0, 0), 3.0);
    return gray_f;
}

// 代价图 (H,B) 的最小累计路径（自顶向下，每步左/中/右，转移罚 lam）
std::vector<int> dpSeam(const cv::Mat& cost, double lam) {
    const int height = cost.rows, width = cost.cols;
    cv::Mat dp(height, width, CV_64F), back(height, width, CV_32S);
    for (int k = 0; k < width; ++k) {
        dp.at<double>(0, k) = cost.at<double>(0, k);
        back.at<int>(0, k) = k;
    }
    for (int r = 1; r < height; ++r) {
        for (int k = 0; k < width; ++k) {
            double best = dp.at<double>(r - 1, k);
            int best_k = k;
            if (k > 0 && dp.at<double>(r - 1, k - 1) + lam < best) {
                best = dp.at<double>(r - 1, k - 1) + lam;
                best_k = k - 1;
            }
            if (k < width - 1 && dp.at<double>(r - 1, k + 1) + lam < best) {
                best = dp.at<double>(r - 1, k + 1) + lam;
                best_k = k + 1;
            }
            dp.at<double>(r, k) = cost.at<double>(r, k) + best;
            back.at<int>(r, k) = best_k;
        }
    }

    std::vector<int> path(height);
    int best_k = 0;
    double best_value = dp.at<double>(height - 1, 0);
    for (int k = 1; k < width; ++k) {
        const double value = dp.at<double>(height - 1, k);
        if (value < best_value) {
            best_value = value;
            best_k = k;
        }
    }
    path[height - 1] = best_k;
    for (int r = height - 1; r > 0; --r)
        path[r - 1] = back.at<int>(r, path[r]);
    return path;
}

void checkBlendInputs(const std::vector<cv::Mat>& warps, const std::vector<cv::Mat>& weights,
                      const char* name) {
    if (warps.empty() || warps.size() != weights.size()) {
        throw std::runtime_error(std::string(name) + ": warps/weights size mismatch");
    }
    for (size_t i = 0; i < warps.size(); ++i) {
        if (warps[i].empty() || weights[i].empty() || warps[i].size() != weights[i].size()) {
            throw std::runtime_error(std::string(name) + ": invalid warp/weight matrix");
        }
    }
}

} // namespace

cv::Mat truncToU8(const cv::Mat& src) {
    if (src.empty()) throw std::runtime_error("truncToU8: empty input");
    cv::Mat floats;
    src.convertTo(floats, CV_32F);
    cv::Mat out(floats.size(), CV_MAKETYPE(CV_8U, floats.channels()));
    const size_t total = floats.total() * floats.channels();
    const float* in = reinterpret_cast<const float*>(floats.data);
    uchar* dst = out.data;
    for (size_t k = 0; k < total; ++k) {
        float value = in[k];
        if (value < 0.0f) value = 0.0f;
        if (value > 255.0f) value = 255.0f;
        dst[k] = static_cast<uchar>(static_cast<int>(value)); // 向零截断
    }
    return out;
}

cv::Mat featherBlend(const std::vector<cv::Mat>& warps, const std::vector<cv::Mat>& weights) {
    checkBlendInputs(warps, weights, "featherBlend");

    cv::Mat accum = cv::Mat::zeros(warps[0].size(), CV_32FC3);
    cv::Mat weight_sum = cv::Mat::zeros(warps[0].size(), CV_32F);
    for (size_t i = 0; i < warps.size(); ++i) {
        const cv::Mat& weight = weights[i];
        std::vector<cv::Mat> channels = {weight, weight, weight};
        cv::Mat weight3;
        cv::merge(channels, weight3);
        accum += warps[i].mul(weight3);
        weight_sum += weight;
    }

    cv::Mat safe_sum;
    cv::max(weight_sum, 1e-6f, safe_sum);
    std::vector<cv::Mat> accum_channels;
    cv::split(accum, accum_channels);
    for (auto& channel : accum_channels)
        channel = channel / safe_sum;
    cv::Mat out;
    cv::merge(accum_channels, out);
    cv::min(out, 255.0, out);
    cv::max(out, 0.0, out);
    return out;
}

cv::Mat blendMultiband(const std::vector<cv::Mat>& warps, const std::vector<cv::Mat>& weights,
                       int levels, int threads) {
    checkBlendInputs(warps, weights, "blendMultiband");
    if (levels < 1) throw std::runtime_error("blendMultiband: levels must be >= 1");

    const int n = static_cast<int>(warps.size());
    cv::Mat weight_sum = cv::Mat::zeros(warps[0].size(), CV_32F);
    for (int i = 0; i < n; ++i)
        weight_sum += weights[i]; // 保持累加顺序
    cv::max(weight_sum, 1e-6, weight_sum);

    // 每台相机：归一化权重 + 建 Laplacian/Gaussian 金字塔（互不依赖 → 并行）
    std::vector<std::vector<cv::Mat>> laplacian(n), gaussian(n);
    parallel_for(n, threads, [&](int i) {
        const cv::Mat normalized = weights[i] / weight_sum;
        laplacian[i] = laplacianPyramid(warps[i], levels);
        gaussian[i].push_back(normalized);
        for (int l = 1; l < levels; ++l) {
            cv::Mat down;
            cv::pyrDown(gaussian[i][l - 1], down);
            gaussian[i].push_back(down);
        }
    });

    // 各层独立 → 并行
    std::vector<cv::Mat> blended(levels);
    parallel_for(levels, threads, [&](int l) {
        cv::Mat accum = cv::Mat::zeros(laplacian[0][l].size(), CV_32FC3);
        cv::Mat gaussian_sum = cv::Mat::zeros(gaussian[0][l].size(), CV_32F);
        for (int i = 0; i < n; ++i) { // 相机按序累加，保证逐位一致
            std::vector<cv::Mat> channels = {gaussian[i][l], gaussian[i][l], gaussian[i][l]};
            cv::Mat gaussian3;
            cv::merge(channels, gaussian3);
            accum += laplacian[i][l].mul(gaussian3);
            gaussian_sum += gaussian[i][l];
        }
        cv::max(gaussian_sum, 1e-6, gaussian_sum);
        std::vector<cv::Mat> accum_channels;
        cv::split(accum, accum_channels);
        for (auto& channel : accum_channels)
            channel = channel / gaussian_sum;
        cv::merge(accum_channels, blended[l]);
    });

    cv::Mat out = blended[levels - 1];
    for (int l = levels - 2; l >= 0; --l) {
        cv::Mat up;
        cv::pyrUp(out, up, blended[l].size());
        out = up + blended[l];
    }
    return out;
}

std::vector<cv::Mat> computeSeamWeights(const std::vector<cv::Mat>& warps,
                                        const std::vector<cv::Mat>& valids,
                                        const std::vector<double>& centerAngles, double fov_h_deg,
                                        double feather_px, double band_frac, double lam,
                                        int threads) {
    if (warps.empty() || warps.size() != valids.size() || warps.size() != centerAngles.size()) {
        throw std::runtime_error("computeSeamWeights: input size mismatch");
    }
    if (!(fov_h_deg > 0.0)) {
        throw std::runtime_error("computeSeamWeights: invalid field of view");
    }
    if (warps[0].empty()) throw std::runtime_error("computeSeamWeights: empty warp");

    const int n = static_cast<int>(warps.size());
    const int H = warps[0].rows, W = warps[0].cols;
    const double f = W / (fov_h_deg * kPi / 180.0);

    // 列水平角（度）
    std::vector<double> theta(W);
    for (int u = 0; u < W; ++u)
        theta[u] = ((u + 0.5) - W / 2.0) / f * 180.0 / kPi;

    // 基础归属：最近中心
    cv::Mat owner(H, W, CV_32S);
    for (int u = 0; u < W; ++u) {
        int best_i = 0;
        double best_d = std::fabs(wrapAngleDeg(theta[u] - centerAngles[0]));
        for (int i = 1; i < n; ++i) {
            const double d = std::fabs(wrapAngleDeg(theta[u] - centerAngles[i]));
            if (d < best_d) {
                best_d = d;
                best_i = i;
            }
        }
        for (int v = 0; v < H; ++v)
            owner.at<int>(v, u) = best_i;
    }

    // 灰度图（每相机一次；互不依赖 → 并行）
    std::vector<cv::Mat> gray(n);
    parallel_for(n, threads, [&](int i) { gray[i] = grayOf(warps[i]); });

    // 各相邻对独立、且覆盖的输出列互不重叠 → 并行找缝
    parallel_for(n, threads, [&](int i) {
        const int j = (i + 1) % n;
        const double ca_i = centerAngles[i], ca_j = centerAngles[j];
        double delta = wrapAngleDeg(ca_j - ca_i);
        if (delta <= 0) delta += 360.0;

        std::vector<int> band;
        for (int u = 0; u < W; ++u) {
            double s = wrapAngleDeg(theta[u] - ca_i);
            if (s < 0) s += 360.0;
            if (s >= band_frac * 0.5 * delta && s <= delta - band_frac * 0.5 * delta) {
                band.push_back(u);
            }
        }

        // 过滤：两相机该列都需有有效像素
        std::vector<int> valid_band;
        for (int u : band) {
            bool a = false, b = false;
            for (int v = 0; v < H && !(a && b); ++v) {
                if (valids[i].at<uchar>(v, u)) a = true;
                if (valids[j].at<uchar>(v, u)) b = true;
            }
            if (a && b) valid_band.push_back(u);
        }
        if (static_cast<int>(valid_band.size()) < 4) return;

        const int B = static_cast<int>(valid_band.size());
        cv::Mat cost(H, B, CV_64F);
        for (int k = 0; k < B; ++k) {
            const int u = valid_band[k];
            for (int v = 0; v < H; ++v) {
                const bool ok = valids[i].at<uchar>(v, u) && valids[j].at<uchar>(v, u);
                cost.at<double>(v, k) =
                    ok ? std::fabs(gray[i].at<float>(v, u) - gray[j].at<float>(v, u)) : 1e6;
            }
        }

        const std::vector<int> path = dpSeam(cost, lam);
        for (int k = 0; k < B; ++k) {
            const int u = valid_band[k];
            for (int v = 0; v < H; ++v) {
                owner.at<int>(v, u) = (k < path[v]) ? i : j;
            }
        }
    });

    // 组装权重 + 羽化（每相机独立 → 并行）
    std::vector<cv::Mat> weights(n);
    parallel_for(n, threads, [&](int i) {
        cv::Mat wi(H, W, CV_32F, cv::Scalar(0));
        for (int v = 0; v < H; ++v) {
            for (int u = 0; u < W; ++u) {
                wi.at<float>(v, u) =
                    (owner.at<int>(v, u) == i && valids[i].at<uchar>(v, u)) ? 1.0f : 0.0f;
            }
        }
        if (feather_px > 0) {
            cv::GaussianBlur(wi, wi, cv::Size(0, 0), feather_px);
            for (int v = 0; v < H; ++v) {
                for (int u = 0; u < W; ++u) {
                    if (!valids[i].at<uchar>(v, u)) wi.at<float>(v, u) = 0.0f;
                }
            }
        }
        weights[i] = wi;
    });

    // 归一化（求和保持顺序，逐位一致）
    cv::Mat weight_sum = cv::Mat::zeros(H, W, CV_32F);
    for (int i = 0; i < n; ++i)
        weight_sum += weights[i];
    cv::max(weight_sum, 1e-6, weight_sum);
    for (int i = 0; i < n; ++i)
        weights[i] = weights[i] / weight_sum;
    return weights;
}

} // namespace pan
