// blend.cpp —— 融合实现
#include <panostitch/blend.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <opencv2/imgproc.hpp>
#include <panostitch/parallel.hpp>

namespace pan {

namespace {

constexpr double kPi = 3.14159265358979323846;

double wrapAngleDeg(double a) {
    double r = std::fmod(a + 180.0, 360.0);
    if (r < 0) r += 360.0;
    return r - 180.0;
}

std::vector<cv::Mat> laplacianPyramid(const cv::Mat& img, int levels) {
    std::vector<cv::Mat> g;
    g.push_back(img.clone());
    for (int i = 1; i < levels; ++i) {
        cv::Mat down; cv::pyrDown(g[i - 1], down); g.push_back(down);
    }
    std::vector<cv::Mat> lap(levels);
    for (int i = 0; i < levels - 1; ++i) {
        cv::Mat up;
        cv::pyrUp(g[i + 1], up, g[i].size());
        lap[i] = g[i] - up;
    }
    lap[levels - 1] = g[levels - 1];
    return lap;
}

cv::Mat grayOf(const cv::Mat& warp) {
    cv::Mat u8 = truncToU8(warp);
    cv::Mat gray, gf;
    cv::cvtColor(u8, gray, cv::COLOR_BGR2GRAY);
    gray.convertTo(gf, CV_32F);
    cv::GaussianBlur(gf, gf, cv::Size(0, 0), 3.0);
    return gf;
}

// 代价图 (H,B) 的最小累计路径（自顶向下，每步左/中/右，转移罚 lam）
std::vector<int> dpSeam(const cv::Mat& cost, double lam) {
    const int h = cost.rows, b = cost.cols;
    cv::Mat dp(h, b, CV_64F), bk(h, b, CV_32S);
    for (int k = 0; k < b; ++k) { dp.at<double>(0, k) = cost.at<double>(0, k); bk.at<int>(0, k) = k; }
    for (int r = 1; r < h; ++r) {
        for (int k = 0; k < b; ++k) {
            double best = dp.at<double>(r - 1, k); int bd = k;
            if (k > 0 && dp.at<double>(r - 1, k - 1) + lam < best) { best = dp.at<double>(r - 1, k - 1) + lam; bd = k - 1; }
            if (k < b - 1 && dp.at<double>(r - 1, k + 1) + lam < best) { best = dp.at<double>(r - 1, k + 1) + lam; bd = k + 1; }
            dp.at<double>(r, k) = cost.at<double>(r, k) + best;
            bk.at<int>(r, k) = bd;
        }
    }
    std::vector<int> path(h);
    int best = 0; double bv = dp.at<double>(h - 1, 0);
    for (int k = 1; k < b; ++k) { double v = dp.at<double>(h - 1, k); if (v < bv) { bv = v; best = k; } }
    path[h - 1] = best;
    for (int r = h - 1; r > 0; --r) path[r - 1] = bk.at<int>(r, path[r]);
    return path;
}

} // namespace

cv::Mat truncToU8(const cv::Mat& src) {
    cv::Mat ff;
    src.convertTo(ff, CV_32F);
    cv::Mat out(ff.size(), CV_MAKETYPE(CV_8U, ff.channels()));
    const size_t total = ff.total() * ff.channels();
    const float* p = reinterpret_cast<const float*>(ff.data);
    uchar* q = out.data;
    for (size_t k = 0; k < total; ++k) {
        float v = p[k];
        if (v < 0.f) v = 0.f;
        if (v > 255.f) v = 255.f;
        q[k] = static_cast<uchar>(static_cast<int>(v));   // 向零截断
    }
    return out;
}

cv::Mat featherBlend(const std::vector<cv::Mat>& warps, const std::vector<cv::Mat>& weights) {
    cv::Mat acc = cv::Mat::zeros(warps[0].size(), CV_32FC3);
    cv::Mat wsum = cv::Mat::zeros(warps[0].size(), CV_32F);
    for (size_t i = 0; i < warps.size(); ++i) {
        cv::Mat w3;
        cv::Mat w = weights[i];
        std::vector<cv::Mat> ch = {w, w, w};
        cv::merge(ch, w3);
        acc += warps[i].mul(w3);
        wsum += w;
    }
    std::vector<cv::Mat> accc; cv::split(acc, accc);
    for (auto& c : accc) {
        cv::Mat ws;
        cv::max(wsum, 1e-6f, ws);
        c = c / ws;
    }
    cv::Mat out; cv::merge(accc, out);
    cv::min(out, 255.0, out); cv::max(out, 0.0, out);
    return out;
}

cv::Mat blendMultiband(const std::vector<cv::Mat>& warps,
                       const std::vector<cv::Mat>& weights, int levels, int threads) {
    const int n = (int)warps.size();
    cv::Mat wsum = cv::Mat::zeros(warps[0].size(), CV_32F);
    for (int i = 0; i < n; ++i) wsum += weights[i];   // 保持累加顺序
    cv::max(wsum, 1e-6, wsum);

    // 每台相机：归一化权重 + 建 Laplacian/Gaussian 金字塔（互不依赖 → 并行）
    std::vector<std::vector<cv::Mat>> L(n), G(n);
    parallel_for(n, threads, [&](int i) {
        cv::Mat wn = weights[i] / wsum;
        L[i] = laplacianPyramid(warps[i], levels);
        std::vector<cv::Mat> gj; gj.push_back(wn);
        G[i] = gj;
        for (int l = 1; l < levels; ++l) { cv::Mat d; cv::pyrDown(G[i][l - 1], d); G[i].push_back(d); }
    });

    // 各层独立 → 并行
    std::vector<cv::Mat> blended(levels);
    parallel_for(levels, threads, [&](int l) {
        cv::Mat acc = cv::Mat::zeros(L[0][l].size(), CV_32FC3);
        cv::Mat gs = cv::Mat::zeros(G[0][l].size(), CV_32F);
        for (int i = 0; i < n; ++i) {                 // 相机按序累加，保证逐位一致
            cv::Mat g3; std::vector<cv::Mat> ch = {G[i][l], G[i][l], G[i][l]}; cv::merge(ch, g3);
            acc += L[i][l].mul(g3);
            gs += G[i][l];
        }
        cv::max(gs, 1e-6, gs);
        std::vector<cv::Mat> accc; cv::split(acc, accc);
        for (auto& c : accc) { c = c / gs; }
        cv::merge(accc, blended[l]);
    });
    cv::Mat out = blended[levels - 1];
    for (int l = levels - 2; l >= 0; --l) {
        cv::Mat up; cv::pyrUp(out, up, blended[l].size());
        out = up + blended[l];
    }
    return out;
}

std::vector<cv::Mat> computeSeamWeights(const std::vector<cv::Mat>& warps,
                                        const std::vector<cv::Mat>& valids,
                                        const std::vector<double>& centerAngles,
                                        double fov_h_deg,
                                        double feather_px, double band_frac, double lam,
                                        int threads) {
    const int n = (int)warps.size();
    const int H = warps[0].rows, W = warps[0].cols;
    const double f = W / (fov_h_deg * kPi / 180.0);

    // 列水平角（度）
    std::vector<double> theta(W);
    for (int u = 0; u < W; ++u) theta[u] = ((u + 0.5) - W / 2.0) / f * 180.0 / kPi;

    // 基础归属：最近中心
    cv::Mat owner(H, W, CV_32S);
    for (int u = 0; u < W; ++u) {
        int bi = 0; double bd = std::fabs(wrapAngleDeg(theta[u] - centerAngles[0]));
        for (int i = 1; i < n; ++i) { double d = std::fabs(wrapAngleDeg(theta[u] - centerAngles[i])); if (d < bd) { bd = d; bi = i; } }
        for (int v = 0; v < H; ++v) owner.at<int>(v, u) = bi;
    }

    // 灰度图（每相机一次；互不依赖 → 并行）
    std::vector<cv::Mat> gray(n);
    parallel_for(n, threads, [&](int i) { gray[i] = grayOf(warps[i]); });

    // 各相邻对独立、且覆盖的输出列互不重叠 → 并行找缝
    parallel_for(n, threads, [&](int i) {
        int j = (i + 1) % n;
        double ca_i = centerAngles[i], ca_j = centerAngles[j];
        double delta = wrapAngleDeg(ca_j - ca_i); if (delta <= 0) delta += 360.0;
        if (delta <= 0) return;
        std::vector<int> band;
        for (int u = 0; u < W; ++u) {
            double s = wrapAngleDeg(theta[u] - ca_i); if (s < 0) s += 360.0;
            if (s >= band_frac * 0.5 * delta && s <= delta - band_frac * 0.5 * delta) band.push_back(u);
        }
        // 过滤：两相机该列都需有有效像素
        std::vector<int> band2;
        for (int u : band) {
            bool a = false, b = false;
            for (int v = 0; v < H && !(a && b); ++v) {
                if (valids[i].at<uchar>(v, u)) a = true;
                if (valids[j].at<uchar>(v, u)) b = true;
            }
            if (a && b) band2.push_back(u);
        }
        if ((int)band2.size() < 4) return;
        const int B = (int)band2.size();
        cv::Mat cost(H, B, CV_64F);
        for (int k = 0; k < B; ++k) {
            int u = band2[k];
            for (int v = 0; v < H; ++v) {
                bool ok = valids[i].at<uchar>(v, u) && valids[j].at<uchar>(v, u);
                cost.at<double>(v, k) = ok ? std::fabs(gray[i].at<float>(v, u) - gray[j].at<float>(v, u)) : 1e6;
            }
        }
        std::vector<int> path = dpSeam(cost, lam);
        for (int k = 0; k < B; ++k) {
            int u = band2[k];
            for (int v = 0; v < H; ++v) owner.at<int>(v, u) = (k < path[v]) ? i : j;
        }
    });

    // 组装权重 + 羽化（每相机独立 → 并行）
    std::vector<cv::Mat> weights(n);
    parallel_for(n, threads, [&](int i) {
        cv::Mat wi(H, W, CV_32F, cv::Scalar(0));
        for (int v = 0; v < H; ++v) for (int u = 0; u < W; ++u)
            wi.at<float>(v, u) = (owner.at<int>(v, u) == i && valids[i].at<uchar>(v, u)) ? 1.0f : 0.0f;
        if (feather_px > 0) {
            cv::GaussianBlur(wi, wi, cv::Size(0, 0), feather_px);
            for (int v = 0; v < H; ++v) for (int u = 0; u < W; ++u)
                if (!valids[i].at<uchar>(v, u)) wi.at<float>(v, u) = 0.0f;
        }
        weights[i] = wi;
    });
    // 归一化（求和保持顺序，逐位一致）
    cv::Mat ssum = cv::Mat::zeros(H, W, CV_32F);
    for (int i = 0; i < n; ++i) ssum += weights[i];
    cv::max(ssum, 1e-6, ssum);
    for (int i = 0; i < n; ++i) weights[i] = weights[i] / ssum;
    return weights;
}

} // namespace pan
