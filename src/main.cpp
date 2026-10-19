// main.cpp —— 单张全景合成器（对齐 Python tools/stitch_preview.py）
//
// 流程：读产物 → 光度(gain) → remap →（找缝）→ 融合 → 保存，并打印分阶段耗时。
// 支持 --parallax lut（读已烘焙 LUT）或 inmem（内存构造，含焦点/地面视差补偿）。
//
// 用法：
//   panostitch --frames data/preview --blend multiband --seam
//   panostitch --frames data/preview --parallax inmem --ground-height 0.5 --blend multiband --seam
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "config.hpp"
#include "calib_io.hpp"
#include "npy_io.hpp"
#include "parallax.hpp"
#include "blend.hpp"
#include "parallel.hpp"
#include "paths.hpp"

using namespace pan;
namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;

static double ms(Clock::time_point a, Clock::time_point b) {
    return std::chrono::duration<double, std::milli>(b - a).count();
}

// 按命名约定查找第 i 路相机的一帧图像（0.* / cam0.* / cam_0.* / 00.*）
static std::string findFrame(const std::string& dir, int i) {
    std::vector<std::string> pats;
    pats.push_back(std::to_string(i) + ".*");
    pats.push_back("cam" + std::to_string(i) + ".*");
    pats.push_back("cam_" + std::to_string(i) + ".*");
    char buf[16]; std::snprintf(buf, sizeof(buf), "%02d.*", i); pats.push_back(buf);
    for (const auto& p : pats) {
        std::vector<cv::String> hits;
        cv::glob(dir + "/" + p, hits, false);
        if (!hits.empty()) { std::sort(hits.begin(), hits.end()); return hits[0]; }
    }
    return "";
}

struct Cam {
    cv::Mat mapx, mapy;      // CV_32F
    cv::Mat weight;          // CV_32F (H,W)
    cv::Mat gain;            // CV_32F (3,1) 或空
    cv::Mat vignette;        // 可选
    double  center_angle = 0;// 度
    Intrinsics intr;
    Extrinsics extr;
};

int main(int argc, char** argv) {
    // ---- 参数 ----
    std::string frames_dir, out_path, config_path;
    std::string blend = "multiband";
    std::string parallax = "lut";
    bool seam = false;
    int levels = 5;
    double focus_m = -1, ground_m = -1, far_m = -1, max_ground_m = -1;
    int threads = 1;
    int dump_warp = -1; std::string dump_path; std::string dump_pano;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&](double def) -> double { return (i + 1 < argc) ? std::atof(argv[++i]) : def; };
        if (a == "--frames") frames_dir = argv[++i];
        else if (a == "--out") out_path = argv[++i];
        else if (a == "--config") config_path = argv[++i];
        else if (a == "--blend") blend = argv[++i];
        else if (a == "--parallax") parallax = argv[++i];
        else if (a == "--seam") seam = true;
        else if (a == "--levels") levels = std::atoi(argv[++i]);
        else if (a == "--threads") threads = std::atoi(argv[++i]);
        else if (a == "--focus-distance") focus_m = next(-1);
        else if (a == "--ground-height") ground_m = next(-1);
        else if (a == "--far-distance") far_m = next(-1);
        else if (a == "--max-ground-distance") max_ground_m = next(-1);
        else if (a == "--dump-warp") { dump_warp = std::atoi(argv[++i]); dump_path = argv[++i]; }
        else if (a == "--dump-pano") { dump_pano = argv[++i]; }
        else { std::cerr << "Unknown argument: " << a << "\n"; return 2; }
    }

    // 多线程版本：--threads N（0=取硬件线程数）；默认 1（单线程，与 Python 一致）
    if (threads == 0) threads = static_cast<int>(std::thread::hardware_concurrency());
    if (threads < 1) threads = 1;
    cv::setNumThreads(1);   // OpenCV 内部仍单线程，并行度由本程序自己控制（保证可复现）

    const fs::path app_root = findAppRoot();
    if (config_path.empty()) config_path = (app_root / "config.json").string();
    const fs::path config_abs = fs::absolute(fs::path(config_path));
    const fs::path config_dir = config_abs.parent_path();

    Config cfg = loadConfig(config_abs.string());
    const std::string calib_root = resolvePath(config_dir, fs::path(cfg.calib_root)).string();
    const std::string frames_root = resolvePath(config_dir, fs::path(cfg.frames_root)).string();
    const std::string out_root = resolvePath(config_dir, fs::path(cfg.out_root)).string();

    const int n = cfg.count;
    const double fov = cfg.fov_h_deg;
    const int W = cfg.out_w, H = cfg.out_h;
    if (frames_dir.empty()) frames_dir = frames_root;
    if (out_path.empty()) out_path = (fs::path(out_root) / "cpp_pano.jpg").string();

    // ---- 读产物 ----
    auto t_read0 = Clock::now();
    std::vector<Cam> cams(n);
    for (int i = 0; i < n; ++i) {
        std::string cd = calib_root + "/camera_" + std::to_string(i);
        Cam& c = cams[i];
        c.weight = loadWeight(cd + "/weight.bin");
        try { c.gain = loadNpy(cd + "/gain.npy"); } catch (...) { c.gain = cv::Mat(); }
        c.extr = loadExtrinsicsYml(cd + "/extrinsics.yml");
        // 中心方位角：光轴 (0,0,1) 转到世界系后取 atan2(x,z)
        cv::Mat axis = c.extr.Rcw * (cv::Mat_<double>(3, 1) << 0, 0, 1);
        c.center_angle = std::atan2(axis.at<double>(0), axis.at<double>(2)) * 180.0 / CV_PI;
    }
    auto t_read1 = Clock::now();
    double t_read = ms(t_read0, t_read1);

    // ---- 读源帧 ----
    auto tf0 = Clock::now();
    std::vector<cv::Mat> frames(n);
    for (int i = 0; i < n; ++i) {
        std::string p = findFrame(frames_dir, i);
        if (p.empty()) {
            std::cerr << "[error] no frame for camera " << i << " in " << frames_dir << "\n";
            return 1;
        }
        frames[i] = cv::imread(p, cv::IMREAD_COLOR);
        std::cout << "source image[" << i << "]: " << p << "\n";
    }
    double t_frames = ms(tf0, Clock::now());

    // ---- 视差映射 ----
    double t_map = 0;
    std::string mode = "LUT (baked / pure rotation)";
    if (parallax == "inmem") {
        // 命令行给了 H 或 D 就以命令行为准；都没给才读 config（地面优先，与 Python 一致）
        double gm, fm;
        if (ground_m < 0 && focus_m < 0) { gm = cfg.ground_height_m; fm = cfg.focus_distance_m; }
        else { gm = (ground_m >= 0) ? ground_m : 0.0; fm = (focus_m >= 0) ? focus_m : 0.0; }
        double far = (far_m >= 0) ? far_m : cfg.far_distance_m;
        double mg = (max_ground_m >= 0) ? max_ground_m : cfg.max_ground_m;
        char mbuf[64];
        if (gm > 0)      { std::snprintf(mbuf, sizeof(mbuf), "inmem ground H=%.2fm", gm); mode = mbuf; }
        else if (fm > 0) { std::snprintf(mbuf, sizeof(mbuf), "inmem focus D=%.2fm", fm); mode = mbuf; }
        else             mode = "inmem pure rotation";

        auto tm0 = Clock::now();
        cv::Mat dirs; double f;
        buildWorldDirs(W, H, fov, dirs, f);   // 共享只读
        const double gmm = gm * 1000.0, fmm = fm * 1000.0, frm = far * 1000.0, mmm = mg * 1000.0;
        // 各相机映射互不依赖 → 并行
        parallel_for(n, threads, [&](int i) {
            cams[i].intr = loadIntrinsicsYml(calib_root + "/camera_" + std::to_string(i) + "/intrinsics.yml");
            if (gmm > 0)
                projectCamera(dirs, W, H, cams[i].intr, cams[i].extr, 0.0, gmm, frm, mmm, cams[i].mapx, cams[i].mapy);
            else if (fmm > 0)
                projectCamera(dirs, W, H, cams[i].intr, cams[i].extr, fmm, 0.0, 0.0, 0.0, cams[i].mapx, cams[i].mapy);
            else
                projectCamera(dirs, W, H, cams[i].intr, cams[i].extr, 0.0, 0.0, 0.0, 0.0, cams[i].mapx, cams[i].mapy);
        });
        t_map = ms(tm0, Clock::now());
    } else {
        for (int i = 0; i < n; ++i) {
            Lut lut = loadLut(calib_root + "/camera_" + std::to_string(i) + "/lut.bin");
            cams[i].mapx = lut.mapx; cams[i].mapy = lut.mapy;
        }
    }

    // ---- 光度 + remap ----
    auto tw0 = Clock::now();
    std::vector<cv::Mat> warps(n), valids(n);
    // 各相机光度+remap 互不依赖 → 并行
    parallel_for(n, threads, [&](int i) {
        cv::Mat img; frames[i].convertTo(img, CV_32FC3);
        if (!cams[i].gain.empty()) {
            std::vector<cv::Mat> ch; cv::split(img, ch);
            for (int k = 0; k < 3; ++k) ch[k] *= cams[i].gain.at<float>(k, 0);
            cv::merge(ch, img);
        }
        if (!cams[i].vignette.empty()) cv::multiply(img, cams[i].vignette, img);
        cv::min(img, 255.0, img); cv::max(img, 0.0, img);

        cv::Mat warped;
        cv::remap(img, warped, cams[i].mapx, cams[i].mapy, cv::INTER_LINEAR,
                  cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
        warps[i] = warped;
        cv::Mat valid;
        cv::compare(cams[i].mapx, 0.0f, valid, cv::CMP_GE);   // mapx>=0
        valids[i] = valid;
    });
    double t_warp = ms(tw0, Clock::now());

    if (dump_warp >= 0 && dump_warp < n) {
        std::ofstream ofs(dump_path, std::ios::binary);
        const cv::Mat& wmat = warps[dump_warp];
        cv::Mat cont = wmat.isContinuous() ? wmat : wmat.clone();
        ofs.write(reinterpret_cast<const char*>(cont.data),
                  (std::streamsize)cont.total() * 3 * sizeof(float));
        std::cerr << "[dump] warp " << dump_warp << " -> " << dump_path << "\n";
    }

    // ---- 权重 ----
    auto tb0 = Clock::now();
    std::vector<cv::Mat> weights(n);
    double t_seam = 0, t_blend = 0;
    if (seam) {
        auto ts0 = Clock::now();
        std::vector<double> angles(n);
        for (int i = 0; i < n; ++i) angles[i] = cams[i].center_angle;
        weights = computeSeamWeights(warps, valids, angles, fov, 10.0, 0.6, 1.5, threads);
        t_seam = ms(ts0, Clock::now());
    } else {
        for (int i = 0; i < n; ++i) weights[i] = cams[i].weight;
        t_seam = ms(tb0, Clock::now());
    }

    // ---- 融合 ----
    auto tz0 = Clock::now();
    cv::Mat pano;
    if (blend == "multiband") pano = blendMultiband(warps, weights, levels, threads);
    else pano = featherBlend(warps, weights);
    t_blend = ms(tz0, Clock::now());
    double t_stitch = t_warp + t_seam + t_blend;

    // ---- 保存 ----
    if (!dump_pano.empty()) {
        std::ofstream ofs(dump_pano, std::ios::binary);
        cv::Mat cont = pano.isContinuous() ? pano : pano.clone();
        ofs.write(reinterpret_cast<const char*>(cont.data),
                  (std::streamsize)cont.total() * 3 * sizeof(float));
        std::cerr << "[dump] pano(float) -> " << dump_pano << "\n";
    }

    const fs::path out_parent = fs::path(out_path).parent_path();
    if (!out_parent.empty()) {
        std::error_code ec;
        fs::create_directories(out_parent, ec);
    }

    auto tp0 = Clock::now();
    cv::Mat out8 = truncToU8(pano);   // clip + 向零截断，与 numpy astype(uint8) 一致
    cv::imwrite(out_path, out8);
    double t_save = ms(tp0, Clock::now());

    // ---- 报告（英文/ASCII，避免 Windows 控制台编码乱码）----
    std::cout << "blend " << blend << (seam ? "+seam" : "") << " | parallax " << mode
              << " | threads " << threads << "\n";
    std::cout << "---- single-frame time (ms, " << W << "x" << H << ") ----\n";
    std::printf("  load products   %8.1f\n", t_read + t_frames);
    std::printf("  build maps      %8.1f\n", t_map);
    std::printf("  photometric+remap %6.1f\n", t_warp);
    std::printf("  seam            %8.1f\n", t_seam);
    std::printf("  blend           %8.1f\n", t_blend);
    std::printf("  save            %8.1f\n", t_save);
    std::printf("  stitch total    %8.1f\n", t_stitch);
    std::printf("  total (incl I/O)%8.1f\n", t_read + t_frames + t_map + t_stitch + t_save);
    std::cout << "saved: " << out_path << "\n";
    return 0;
}
