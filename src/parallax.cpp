// parallax.cpp —— 世界方向 & 视差补偿投影实现
#include "parallax.hpp"

#include <cmath>
#include <limits>
#include <opencv2/core.hpp>

namespace pan {

namespace {
constexpr double kPi = 3.14159265358979323846;

// 鱼眼畸变曲线回折前的最大入射角 θ_fold（与 Python fisheye.theta_fold 一致）
double thetaFold(const cv::Mat& D) {
    const double d0 = D.at<double>(0), d1 = D.at<double>(1),
                 d2 = D.at<double>(2), d3 = D.at<double>(3);
    double bestT = 0.0, bestV = -1e300;
    const int n = 4000;
    for (int i = 0; i < n; ++i) {
        double th = 1e-4 + (2.5 - 1e-4) * i / (n - 1);
        double t2 = th * th;
        double td = th * (1.0 + d0 * t2 + d1 * t2 * t2 + d2 * t2 * t2 * t2 + d3 * t2 * t2 * t2 * t2);
        if (td > bestV) { bestV = td; bestT = th; }
    }
    return bestT;
}
} // namespace

void buildWorldDirs(int W, int H, double fov_h_deg, cv::Mat& dirs, double& f) {
    f = W / (fov_h_deg * kPi / 180.0);
    dirs.create(H * W, 3, CV_64F);
    for (int v = 0; v < H; ++v) {
        double hgt = ((v + 0.5) - H / 2.0) / f;
        for (int u = 0; u < W; ++u) {
            double theta = ((u + 0.5) - W / 2.0) / f;
            double x = std::sin(theta), y = hgt, z = std::cos(theta);
            double n = std::sqrt(x * x + y * y + z * z);
            double* p = dirs.ptr<double>(v * W + u);
            p[0] = x / n; p[1] = y / n; p[2] = z / n;
        }
    }
}

void projectCamera(const cv::Mat& dirs, int W, int H, const Intrinsics& intr,
                   const Extrinsics& extr, double focus_mm, double ground_mm,
                   double far_mm, double max_ground_mm,
                   cv::Mat& mapx, cv::Mat& mapy) {
    const cv::Mat R = extr.Rcw;                       // 3x3
    cv::Mat P = extr.Tcw.clone();                     // 3x1（相机光心 = 世界系位置）
    const int N = W * H;

    cv::Mat dcam;
    if (ground_mm > 0.0) {
        // 逐像素：朝下(dy>0)命中地面 Y=+H 用交点；否则远景
        //   far_mm>0：X=far_mm·d；far_mm<=0：纯旋转（不减光心）
        const double maxd = (max_ground_mm > 0) ? max_ground_mm : 1e9;
        const double P0 = P.at<double>(0), P1 = P.at<double>(1), P2 = P.at<double>(2);
        const double r00=R.at<double>(0,0), r01=R.at<double>(0,1), r02=R.at<double>(0,2);
        const double r10=R.at<double>(1,0), r11=R.at<double>(1,1), r12=R.at<double>(1,2);
        const double r20=R.at<double>(2,0), r21=R.at<double>(2,1), r22=R.at<double>(2,2);
        dcam.create(N, 3, CV_64F);
        for (int i = 0; i < N; ++i) {
            const double* d = dirs.ptr<double>(i);
            bool hit = false; double t = 0.0;
            if (d[1] > 1e-9) { t = ground_mm / d[1]; hit = (t <= maxd); }
            double vx, vy, vz;
            if (hit)                      { vx = d[0]*t - P0; vy = d[1]*t - P1; vz = d[2]*t - P2; }
            else if (far_mm > 0.0)        { vx = d[0]*far_mm - P0; vy = d[1]*far_mm - P1; vz = d[2]*far_mm - P2; }
            else                          { vx = d[0]; vy = d[1]; vz = d[2]; }   // 远景纯旋转
            dcam.at<double>(i,0) = vx*r00 + vy*r10 + vz*r20;
            dcam.at<double>(i,1) = vx*r01 + vy*r11 + vz*r21;
            dcam.at<double>(i,2) = vx*r02 + vy*r12 + vz*r22;
        }
    } else if (focus_mm > 0.0) {
        cv::Mat xw(N, 3, CV_64F);
        for (int i = 0; i < N; ++i) {
            const double* d = dirs.ptr<double>(i);
            xw.at<double>(i,0) = d[0]*focus_mm - P.at<double>(0);
            xw.at<double>(i,1) = d[1]*focus_mm - P.at<double>(1);
            xw.at<double>(i,2) = d[2]*focus_mm - P.at<double>(2);
        }
        dcam = xw * R;
    } else {
        dcam = dirs * R;   // 纯旋转：d_cam = d_world @ R_cw
    }

    // 内参
    const double fx = intr.K.at<double>(0,0), fy = intr.K.at<double>(1,1);
    const double cx = intr.K.at<double>(0,2), cy = intr.K.at<double>(1,2);
    const double D0 = intr.D.at<double>(0), D1 = intr.D.at<double>(1),
                 D2 = intr.D.at<double>(2), D3 = intr.D.at<double>(3);
    const double thFold = thetaFold(intr.D);

    mapx.create(H, W, CV_32F);
    mapy.create(H, W, CV_32F);
    for (int i = 0; i < N; ++i) {
        const double x = dcam.at<double>(i,0), y = dcam.at<double>(i,1), z = dcam.at<double>(i,2);
        const double r = std::sqrt(x * x + y * y);
        const double theta = std::atan2(r, z);
        const double t2 = theta * theta;
        const double theta_d = theta * (1.0 + D0*t2 + D1*t2*t2 + D2*t2*t2*t2 + D3*t2*t2*t2*t2);
        const double sr = (r < 1e-12) ? 1.0 : r;
        const double u = fx * theta_d * x / sr + cx;
        const double v = fy * theta_d * y / sr + cy;

        bool valid = std::isfinite(u) && std::isfinite(v) && (theta <= thFold)
                     && u >= 0 && u < intr.w && v >= 0 && v < intr.h;
        if (valid && intr.fold_radius > 0) valid = (theta_d < intr.fold_radius);
        if (valid && intr.has_circle && intr.cr > 0) {
            double dx = u - intr.ccx, dy = v - intr.ccy;
            valid = (dx*dx + dy*dy) <= intr.cr * intr.cr;
        }
        const int row = i / W, col = i % W;
        if (valid) { mapx.at<float>(row,col) = (float)u; mapy.at<float>(row,col) = (float)v; }
        else       { mapx.at<float>(row,col) = -1.0f;    mapy.at<float>(row,col) = -1.0f; }
    }
}

} // namespace pan
