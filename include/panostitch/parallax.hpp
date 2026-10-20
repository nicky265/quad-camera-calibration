// parallax.hpp —— 内存构造柱面映射（含视差补偿）
//
// 对应 Python: pipeline/gen_lut.py 的 build_world_dirs() / project_camera()，
// 以及 common/fisheye.py 的 ray_ground_intersect()（本项目分支已把地面/单距离
// 补偿合并进 project_camera）。
//
// 坐标约定（与全工程一致）：
//   世界系 = 相机 0 系，沿用 OpenCV 相机约定（+X 右、+Y 下、+Z 前），故地面在 Y=+H。
//   X_world = R_cw * X_cam + T_cw；相机光心 P = T_cw。
//   鱼眼等距模型：theta = atan2(r,z); theta_d = theta*(1+k1 t^2 + ...)。
#pragma once

#include <vector>
#include <opencv2/core.hpp>
#include <panostitch/calib_io.hpp>

namespace pan {

// 单相机标定信息（内存构造映射所需）
struct CamCalib {
    Intrinsics intr;
    Extrinsics extr;
};

// 生成柱面输出每个像素的世界方向（N×3 CV_64F，N=W*H）与柱面焦距 f。
void buildWorldDirs(int W, int H, double fov_h_deg, cv::Mat& dirs, double& f);

// 把世界方向投影到某相机，返回 (mapx,mapy) CV_32F（H×W，无效为 -1）。
// focus_mm>0 单距离；ground_mm>0 地面；far_mm 地面模式远景距离（0=纯旋转）。
void projectCamera(const cv::Mat& dirs, int W, int H, const Intrinsics& intr,
                   const Extrinsics& extr, double focus_mm, double ground_mm,
                   double far_mm, double max_ground_mm,
                   cv::Mat& mapx, cv::Mat& mapy);

} // namespace pan
