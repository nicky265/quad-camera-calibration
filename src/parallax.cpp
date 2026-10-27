// parallax.cpp —— 世界方向 & 视差补偿投影实现
#include <panostitch/parallax.hpp>

#include <cmath>
#include <stdexcept>
#include <string>

#include <opencv2/core.hpp>

namespace pan {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr int kFoldSamples = 4000;

// 鱼眼畸变曲线回折前的最大入射角 θ_fold（与 Python fisheye.theta_fold 一致）
double thetaFold(const cv::Mat& D) {
    const double* d = D.ptr<double>(0);
    const double d0 = d[0], d1 = d[1], d2 = d[2], d3 = d[3];
    double best_t = 0.0, best_v = -1e300;
    for (int i = 0; i < kFoldSamples; ++i) {
        const double theta = 1e-4 + (2.5 - 1e-4) * i / (kFoldSamples - 1);
        const double t2 = theta * theta;
        const double theta_d =
            theta * (1.0 + d0 * t2 + d1 * t2 * t2 + d2 * t2 * t2 * t2 + d3 * t2 * t2 * t2 * t2);
        if (theta_d > best_v) {
            best_v = theta_d;
            best_t = theta;
        }
    }
    return best_t;
}

} // namespace

void buildWorldDirs(int W, int H, double fov_h_deg, cv::Mat& dirs, double& f) {
    if (W <= 0 || H <= 0 || !(fov_h_deg > 0.0)) {
        throw std::runtime_error("buildWorldDirs: invalid output size or field of view");
    }
    f = W / (fov_h_deg * kPi / 180.0);
    dirs.create(H * W, 3, CV_64F);
    for (int v = 0; v < H; ++v) {
        const double height = ((v + 0.5) - H / 2.0) / f;
        for (int u = 0; u < W; ++u) {
            const double theta = ((u + 0.5) - W / 2.0) / f;
            const double x = std::sin(theta), y = height, z = std::cos(theta);
            const double norm = std::sqrt(x * x + y * y + z * z);
            double* p = dirs.ptr<double>(v * W + u);
            p[0] = x / norm;
            p[1] = y / norm;
            p[2] = z / norm;
        }
    }
}

void projectCamera(const cv::Mat& dirs, int W, int H, const Intrinsics& intr,
                   const Extrinsics& extr, double focus_mm, double ground_mm, double far_mm,
                   double max_ground_mm, cv::Mat& mapx, cv::Mat& mapy) {
    if (dirs.rows != W * H || dirs.cols != 3 || dirs.type() != CV_64F) {
        throw std::runtime_error("projectCamera: world directions have unexpected shape/type");
    }
    if (intr.K.rows != 3 || intr.K.cols != 3 || intr.K.type() != CV_64F || intr.D.total() < 4 ||
        intr.D.type() != CV_64F) {
        throw std::runtime_error("projectCamera: intrinsics are incomplete");
    }
    if (extr.Rcw.rows != 3 || extr.Rcw.cols != 3 || extr.Rcw.type() != CV_64F ||
        extr.Tcw.total() != 3 || extr.Tcw.type() != CV_64F) {
        throw std::runtime_error("projectCamera: extrinsics are incomplete");
    }

    const cv::Mat& R = extr.Rcw; // 3x3
    const cv::Mat& P = extr.Tcw; // 1x3（相机光心 = 世界系位置）
    const int N = W * H;

    cv::Mat dcam;
    if (ground_mm > 0.0) {
        // 逐像素：朝下(dy>0)命中地面 Y=+H 用交点；否则远景
        //   far_mm>0：X=far_mm·d；far_mm<=0：纯旋转（不减光心）
        const double max_dist = (max_ground_mm > 0) ? max_ground_mm : 1e9;
        const double P0 = P.at<double>(0), P1 = P.at<double>(1), P2 = P.at<double>(2);
        const double r00 = R.at<double>(0, 0), r01 = R.at<double>(0, 1), r02 = R.at<double>(0, 2);
        const double r10 = R.at<double>(1, 0), r11 = R.at<double>(1, 1), r12 = R.at<double>(1, 2);
        const double r20 = R.at<double>(2, 0), r21 = R.at<double>(2, 1), r22 = R.at<double>(2, 2);
        dcam.create(N, 3, CV_64F);
        for (int i = 0; i < N; ++i) {
            const double* d = dirs.ptr<double>(i);
            bool hit = false;
            double t = 0.0;
            if (d[1] > 1e-9) {
                t = ground_mm / d[1];
                hit = (t <= max_dist);
            }
            double vx, vy, vz;
            if (hit) {
                vx = d[0] * t - P0;
                vy = d[1] * t - P1;
                vz = d[2] * t - P2;
            } else if (far_mm > 0.0) {
                vx = d[0] * far_mm - P0;
                vy = d[1] * far_mm - P1;
                vz = d[2] * far_mm - P2;
            } else {
                vx = d[0];
                vy = d[1];
                vz = d[2];
            }
            double* out = dcam.ptr<double>(i);
            out[0] = vx * r00 + vy * r10 + vz * r20;
            out[1] = vx * r01 + vy * r11 + vz * r21;
            out[2] = vx * r02 + vy * r12 + vz * r22;
        }
    } else if (focus_mm > 0.0) {
        cv::Mat xw(N, 3, CV_64F);
        for (int i = 0; i < N; ++i) {
            const double* d = dirs.ptr<double>(i);
            double* p = xw.ptr<double>(i);
            p[0] = d[0] * focus_mm - P.at<double>(0);
            p[1] = d[1] * focus_mm - P.at<double>(1);
            p[2] = d[2] * focus_mm - P.at<double>(2);
        }
        dcam = xw * R;
    } else {
        dcam = dirs * R; // 纯旋转：d_cam = d_world @ R_cw
    }

    // 内参
    const double fx = intr.K.at<double>(0, 0), fy = intr.K.at<double>(1, 1);
    const double cx = intr.K.at<double>(0, 2), cy = intr.K.at<double>(1, 2);
    const double* D = intr.D.ptr<double>(0);
    const double D0 = D[0], D1 = D[1], D2 = D[2], D3 = D[3];
    const double th_fold = thetaFold(intr.D);

    mapx.create(H, W, CV_32F);
    mapy.create(H, W, CV_32F);
    for (int i = 0; i < N; ++i) {
        const double* d = dcam.ptr<double>(i);
        const double x = d[0], y = d[1], z = d[2];
        const double r = std::sqrt(x * x + y * y);
        const double theta = std::atan2(r, z);
        const double t2 = theta * theta;
        const double theta_d =
            theta * (1.0 + D0 * t2 + D1 * t2 * t2 + D2 * t2 * t2 * t2 + D3 * t2 * t2 * t2 * t2);
        const double sr = (r < 1e-12) ? 1.0 : r;
        const double u = fx * theta_d * x / sr + cx;
        const double v = fy * theta_d * y / sr + cy;

        bool valid = std::isfinite(u) && std::isfinite(v) && (theta <= th_fold) && u >= 0 &&
                     u < intr.w && v >= 0 && v < intr.h;
        if (valid && intr.fold_radius > 0) valid = (theta_d < intr.fold_radius);
        if (valid && intr.has_circle && intr.cr > 0) {
            const double dx = u - intr.ccx, dy = v - intr.ccy;
            valid = (dx * dx + dy * dy) <= intr.cr * intr.cr;
        }

        const int row = i / W, col = i % W;
        if (valid) {
            mapx.at<float>(row, col) = static_cast<float>(u);
            mapy.at<float>(row, col) = static_cast<float>(v);
        } else {
            mapx.at<float>(row, col) = -1.0f;
            mapy.at<float>(row, col) = -1.0f;
        }
    }
}

} // namespace pan
