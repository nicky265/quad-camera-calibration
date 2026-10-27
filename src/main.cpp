// main.cpp —— 单张全景合成器（对齐 Python tools/stitch_preview.py）
//
// 流程：读产物 → 光度(gain) → remap →（找缝）→ 融合 → 保存，并打印分阶段耗时。
// 支持 --parallax lut（读已烘焙 LUT）或 inmem（内存构造，含焦点/地面视差补偿）。
//
// 用法：
//   panostitch --frames data/frames --blend multiband --seam
//   panostitch --frames data/frames --parallax inmem --ground-height 0.5 --blend multiband --seam
#include <algorithm>
#include <cerrno>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <panostitch/blend.hpp>
#include <panostitch/calib_io.hpp>
#include <panostitch/config.hpp>
#include <panostitch/npy_io.hpp>
#include <panostitch/parallax.hpp>
#include <panostitch/parallel.hpp>
#include <panostitch/paths.hpp>

using namespace pan;
namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;

namespace {

struct Options {
    std::string frames_dir;
    std::string out_path;
    std::string config_path;
    std::string blend = "multiband";
    std::string parallax = "lut";
    bool seam = false;
    bool help = false;
    int levels = 5;
    int threads = 1;
    int dump_warp = -1;
    std::string dump_warp_path;
    std::string dump_pano_path;
    double focus_m = -1.0;
    double ground_m = -1.0;
    double far_m = -1.0;
    double max_ground_m = -1.0;
};

struct Cam {
    cv::Mat mapx, mapy;      // CV_32F
    cv::Mat weight;          // CV_32F (H,W)
    cv::Mat gain;            // CV_32F (3,1)，可为空
    double center_angle = 0; // 度
    Intrinsics intr;
    Extrinsics extr;
};

double msSince(Clock::time_point start) {
    return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

void printUsage(const char* prog) {
    std::cout << "Usage: " << prog << " [options]\n"
              << "  --frames DIR            source frame directory (default: paths.frames_root)\n"
              << "  --out PATH              output image (default: <out_root>/cpp_pano.jpg)\n"
              << "  --config PATH           config.json path (default: auto-detect)\n"
              << "  --blend {feather,multiband}  blend mode (default: multiband)\n"
              << "  --seam                  enable seam finding before blending\n"
              << "  --levels N              multiband pyramid levels (default: 5)\n"
              << "  --parallax {lut,inmem}  mapping source (default: lut)\n"
              << "  --ground-height H       inmem ground compensation height (m)\n"
              << "  --focus-distance D      inmem single-distance compensation (m)\n"
              << "  --far-distance F        inmem ground far distance (m, 0=pure rotation)\n"
              << "  --max-ground-distance M inmem ground max distance (m, default: 30)\n"
              << "  --threads N             worker threads (0=hardware, default: 1)\n"
              << "  --dump-warp I PATH      debug: dump warped camera I as float32\n"
              << "  --dump-pano PATH        debug: dump blended panorama as float32\n"
              << "  -h, --help              show this help\n";
}

bool takeString(int argc, char** argv, int& i, const char* name, std::string& out) {
    if (i + 1 >= argc) {
        std::cerr << "[args] missing value for " << name << "\n";
        return false;
    }
    out = argv[++i];
    return true;
}

bool parseInt(const std::string& text, int& out) {
    if (text.empty()) return false;
    char* end = nullptr;
    errno = 0;
    const long value = std::strtol(text.c_str(), &end, 10);
    if (errno == ERANGE || end == text.c_str() || *end != '\0' || value < INT_MIN ||
        value > INT_MAX) {
        return false;
    }
    out = static_cast<int>(value);
    return true;
}

bool parseDouble(const std::string& text, double& out) {
    if (text.empty()) return false;
    char* end = nullptr;
    errno = 0;
    const double value = std::strtod(text.c_str(), &end);
    if (errno == ERANGE || end == text.c_str() || *end != '\0' || !std::isfinite(value)) {
        return false;
    }
    out = value;
    return true;
}

bool takeInt(int argc, char** argv, int& i, const char* name, int& out) {
    std::string text;
    if (!takeString(argc, argv, i, name, text)) return false;
    if (!parseInt(text, out)) {
        std::cerr << "[args] invalid " << name << ": " << text << "\n";
        return false;
    }
    return true;
}

bool takeDouble(int argc, char** argv, int& i, const char* name, double& out) {
    std::string text;
    if (!takeString(argc, argv, i, name, text)) return false;
    if (!parseDouble(text, out)) {
        std::cerr << "[args] invalid " << name << ": " << text << "\n";
        return false;
    }
    return true;
}

bool parseArgs(int argc, char** argv, Options& opt) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            opt.help = true;
            return true;
        }
        if (arg == "--seam") {
            opt.seam = true;
        } else if (arg == "--frames") {
            if (!takeString(argc, argv, i, "--frames", opt.frames_dir)) return false;
        } else if (arg == "--out") {
            if (!takeString(argc, argv, i, "--out", opt.out_path)) return false;
        } else if (arg == "--config") {
            if (!takeString(argc, argv, i, "--config", opt.config_path)) return false;
        } else if (arg == "--blend") {
            if (!takeString(argc, argv, i, "--blend", opt.blend)) return false;
        } else if (arg == "--parallax") {
            if (!takeString(argc, argv, i, "--parallax", opt.parallax)) return false;
        } else if (arg == "--levels") {
            if (!takeInt(argc, argv, i, "--levels", opt.levels)) return false;
        } else if (arg == "--threads") {
            if (!takeInt(argc, argv, i, "--threads", opt.threads)) return false;
        } else if (arg == "--focus-distance") {
            if (!takeDouble(argc, argv, i, "--focus-distance", opt.focus_m)) return false;
        } else if (arg == "--ground-height") {
            if (!takeDouble(argc, argv, i, "--ground-height", opt.ground_m)) return false;
        } else if (arg == "--far-distance") {
            if (!takeDouble(argc, argv, i, "--far-distance", opt.far_m)) return false;
        } else if (arg == "--max-ground-distance") {
            if (!takeDouble(argc, argv, i, "--max-ground-distance", opt.max_ground_m)) return false;
        } else if (arg == "--dump-warp") {
            if (!takeInt(argc, argv, i, "--dump-warp", opt.dump_warp)) return false;
            if (!takeString(argc, argv, i, "--dump-warp", opt.dump_warp_path)) return false;
            if (opt.dump_warp < 0) {
                std::cerr << "[args] --dump-warp index must be >= 0\n";
                return false;
            }
        } else if (arg == "--dump-pano") {
            if (!takeString(argc, argv, i, "--dump-pano", opt.dump_pano_path)) return false;
        } else {
            std::cerr << "[args] unknown argument: " << arg << "\n";
            printUsage(argv[0]);
            return false;
        }
    }

    if (opt.blend != "feather" && opt.blend != "multiband") {
        std::cerr << "[args] --blend must be feather or multiband\n";
        return false;
    }
    if (opt.parallax != "lut" && opt.parallax != "inmem") {
        std::cerr << "[args] --parallax must be lut or inmem\n";
        return false;
    }
    if (opt.levels < 1) {
        std::cerr << "[args] --levels must be >= 1\n";
        return false;
    }
    return true;
}

// 按命名约定查找第 i 路相机的一帧图像（0.* / cam0.* / cam_0.* / 00.*）
std::string findFrame(const std::string& dir, int i) {
    const std::vector<std::string> patterns = {
        std::to_string(i) + ".*",
        "cam" + std::to_string(i) + ".*",
        "cam_" + std::to_string(i) + ".*",
        cv::format("%02d.*", i),
    };
    for (const auto& pattern : patterns) {
        std::vector<cv::String> hits;
        cv::glob(dir + "/" + pattern, hits, false);
        if (!hits.empty()) {
            std::sort(hits.begin(), hits.end());
            return hits[0];
        }
    }
    return {};
}

void dumpFloatMat(const cv::Mat& mat, const std::string& path) {
    std::ofstream ofs(path, std::ios::binary);
    if (!ofs) throw std::runtime_error("cannot open dump file: " + path);
    const cv::Mat cont = mat.isContinuous() ? mat : mat.clone();
    ofs.write(reinterpret_cast<const char*>(cont.data),
              static_cast<std::streamsize>(cont.total() * cont.channels() * sizeof(float)));
    if (!ofs) throw std::runtime_error("failed to write dump file: " + path);
}

int run(int argc, char** argv) {
    Options opt;
    if (!parseArgs(argc, argv, opt)) return 2;
    if (opt.help) {
        printUsage(argv[0]);
        return 0;
    }

    // 多线程版本：--threads N（0=取硬件线程数）；默认 1（单线程，与 Python 一致）
    if (opt.threads == 0) {
        const unsigned hw = std::thread::hardware_concurrency();
        opt.threads = (hw > 0) ? static_cast<int>(hw) : 1;
    }
    if (opt.threads < 1) opt.threads = 1;
    cv::setNumThreads(1); // OpenCV 内部仍单线程，并行度由本程序自己控制（保证可复现）

    const fs::path app_root = findAppRoot();
    if (opt.config_path.empty()) opt.config_path = (app_root / "config.json").string();
    const fs::path config_abs = fs::absolute(fs::path(opt.config_path));
    const fs::path config_dir = config_abs.parent_path();

    const Config cfg = loadConfig(config_abs.string());
    const std::string calib_root = resolvePath(config_dir, fs::path(cfg.calib_root)).string();
    const std::string frames_root = resolvePath(config_dir, fs::path(cfg.frames_root)).string();
    const std::string out_root = resolvePath(config_dir, fs::path(cfg.out_root)).string();

    const int n = cfg.count;
    const int W = cfg.out_w, H = cfg.out_h;
    const double fov = cfg.fov_h_deg;
    if (n <= 0) throw std::runtime_error("[config] cameras.count must be > 0");
    if (W <= 0 || H <= 0) throw std::runtime_error("[config] pano.out_size must be positive");
    if (opt.dump_warp >= n) throw std::runtime_error("--dump-warp index out of range");
    if (!(fov > 0.0) || fov > 360.0)
        throw std::runtime_error("[config] pano.fov_h_deg must be in (0, 360]");

    // 金字塔层数不能超过图像尺寸允许的上限，否则 pyrDown 会退化到 1x1 以下。
    const int max_levels =
        static_cast<int>(std::floor(std::log2(static_cast<double>(std::min(W, H))))) + 1;
    if (opt.levels > max_levels) {
        std::cerr << "[warn] --levels " << opt.levels << " clamped to " << max_levels << "\n";
        opt.levels = max_levels;
    }

    const std::string frames_dir = opt.frames_dir.empty() ? frames_root : opt.frames_dir;
    const std::string out_path =
        opt.out_path.empty() ? (fs::path(out_root) / "cpp_pano.jpg").string() : opt.out_path;

    // ---- 读产物 ----
    const auto t_read0 = Clock::now();
    std::vector<Cam> cams(n);
    for (int i = 0; i < n; ++i) {
        const fs::path cd = fs::path(calib_root) / ("camera_" + std::to_string(i));
        Cam& c = cams[i];
        c.weight = loadWeight((cd / "weight.bin").string());
        try {
            c.gain = loadNpy((cd / "gain.npy").string());
        } catch (const std::exception& e) {
            c.gain = cv::Mat();
            std::cerr << "[warn] " << e.what() << "\n";
        }
        if (!c.gain.empty() && c.gain.total() != 3) {
            std::cerr << "[warn] gain.npy for camera " << i << " is not 3 floats, ignored\n";
            c.gain = cv::Mat();
        }
        c.extr = loadExtrinsicsYml((cd / "extrinsics.yml").string());
        // 优先用标定文件记录的中心方位角；旧文件没有该字段时由 Rcw 反算。
        if (c.extr.has_angle) {
            c.center_angle = c.extr.center_angle_deg;
        } else {
            const cv::Mat axis = c.extr.Rcw * (cv::Mat_<double>(3, 1) << 0.0, 0.0, 1.0);
            c.center_angle = std::atan2(axis.at<double>(0), axis.at<double>(2)) * 180.0 / CV_PI;
        }
    }
    const double t_read = msSince(t_read0);

    // ---- 读源帧 ----
    const auto tf0 = Clock::now();
    std::vector<cv::Mat> frames(n);
    for (int i = 0; i < n; ++i) {
        const std::string path = findFrame(frames_dir, i);
        if (path.empty()) {
            throw std::runtime_error("no frame for camera " + std::to_string(i) + " in " +
                                     frames_dir);
        }
        frames[i] = cv::imread(path, cv::IMREAD_COLOR);
        if (frames[i].empty()) throw std::runtime_error("cannot read frame: " + path);
        std::cout << "source image[" << i << "]: " << path << "\n";
    }
    const double t_frames = msSince(tf0);

    // ---- 视差映射 ----
    double t_map = 0.0;
    std::string mode = "LUT (baked / pure rotation)";
    if (opt.parallax == "inmem") {
        // 命令行给了 H 或 D 就以命令行为准；都没给才读 config（地面优先，与 Python 一致）
        double gm, fm;
        if (opt.ground_m < 0 && opt.focus_m < 0) {
            gm = cfg.ground_height_m;
            fm = cfg.focus_distance_m;
        } else {
            gm = (opt.ground_m >= 0) ? opt.ground_m : 0.0;
            fm = (opt.focus_m >= 0) ? opt.focus_m : 0.0;
        }
        const double far = (opt.far_m >= 0) ? opt.far_m : cfg.far_distance_m;
        const double mg = (opt.max_ground_m >= 0) ? opt.max_ground_m : cfg.max_ground_m;

        if (gm > 0) {
            mode = cv::format("inmem ground H=%.2fm", gm);
        } else if (fm > 0) {
            mode = cv::format("inmem focus D=%.2fm", fm);
        } else {
            mode = "inmem pure rotation";
        }

        const auto tm0 = Clock::now();
        cv::Mat dirs;
        double cyl_f = 0.0;
        buildWorldDirs(W, H, fov, dirs, cyl_f); // 共享只读
        // 内参读取是串行 I/O，放在并行区之外，避免线程里抛异常。
        for (int i = 0; i < n; ++i) {
            cams[i].intr = loadIntrinsicsYml(
                (fs::path(calib_root) / ("camera_" + std::to_string(i)) / "intrinsics.yml")
                    .string());
        }
        const double gmm = gm * 1000.0, fmm = fm * 1000.0, frm = far * 1000.0, mmm = mg * 1000.0;
        // 各相机映射互不依赖 → 并行
        parallel_for(n, opt.threads, [&](int i) {
            if (gmm > 0) {
                projectCamera(dirs, W, H, cams[i].intr, cams[i].extr, 0.0, gmm, frm, mmm,
                              cams[i].mapx, cams[i].mapy);
            } else if (fmm > 0) {
                projectCamera(dirs, W, H, cams[i].intr, cams[i].extr, fmm, 0.0, 0.0, 0.0,
                              cams[i].mapx, cams[i].mapy);
            } else {
                projectCamera(dirs, W, H, cams[i].intr, cams[i].extr, 0.0, 0.0, 0.0, 0.0,
                              cams[i].mapx, cams[i].mapy);
            }
        });
        t_map = msSince(tm0);
    } else {
        for (int i = 0; i < n; ++i) {
            const fs::path lut_dir = fs::path(calib_root) / ("camera_" + std::to_string(i));
            const Lut lut = loadLut((lut_dir / "lut.bin").string());
            cams[i].mapx = lut.mapx;
            cams[i].mapy = lut.mapy;
        }
    }

    // ---- 光度 + remap ----
    const auto tw0 = Clock::now();
    std::vector<cv::Mat> warps(n), valids(n);
    // 各相机光度+remap 互不依赖 → 并行
    parallel_for(n, opt.threads, [&](int i) {
        cv::Mat img;
        frames[i].convertTo(img, CV_32FC3);
        if (!cams[i].gain.empty()) {
            std::vector<cv::Mat> ch;
            cv::split(img, ch);
            for (int k = 0; k < 3; ++k)
                ch[k] *= cams[i].gain.at<float>(k, 0);
            cv::merge(ch, img);
        }
        cv::min(img, 255.0, img);
        cv::max(img, 0.0, img);

        cv::Mat warped;
        cv::remap(img, warped, cams[i].mapx, cams[i].mapy, cv::INTER_LINEAR, cv::BORDER_CONSTANT,
                  cv::Scalar(0, 0, 0));
        warps[i] = warped;
        cv::compare(cams[i].mapx, 0.0f, valids[i], cv::CMP_GE); // mapx >= 0
    });
    const double t_warp = msSince(tw0);

    if (opt.dump_warp >= 0) {
        dumpFloatMat(warps[opt.dump_warp], opt.dump_warp_path);
        std::cerr << "[dump] warp " << opt.dump_warp << " -> " << opt.dump_warp_path << "\n";
    }

    // ---- 权重 + 融合 ----
    const auto tz0 = Clock::now();
    std::vector<cv::Mat> weights(n);
    double t_seam = 0.0;
    if (opt.seam) {
        const auto ts0 = Clock::now();
        std::vector<double> angles(n);
        for (int i = 0; i < n; ++i)
            angles[i] = cams[i].center_angle;
        weights = computeSeamWeights(warps, valids, angles, fov, 10.0, 0.6, 1.5, opt.threads);
        t_seam = msSince(ts0);
    } else {
        for (int i = 0; i < n; ++i)
            weights[i] = cams[i].weight;
    }

    cv::Mat pano;
    if (opt.blend == "multiband") {
        pano = blendMultiband(warps, weights, opt.levels, opt.threads);
    } else {
        pano = featherBlend(warps, weights);
    }
    const double t_blend = msSince(tz0) - t_seam;
    const double t_stitch = t_warp + t_seam + t_blend;

    // ---- 保存 ----
    if (!opt.dump_pano_path.empty()) {
        dumpFloatMat(pano, opt.dump_pano_path);
        std::cerr << "[dump] pano(float) -> " << opt.dump_pano_path << "\n";
    }

    const fs::path out_parent = fs::path(out_path).parent_path();
    if (!out_parent.empty()) {
        std::error_code ec;
        fs::create_directories(out_parent, ec);
        if (ec) {
            throw std::runtime_error("cannot create output directory " + out_parent.string() +
                                     ": " + ec.message());
        }
    }

    const auto tp0 = Clock::now();
    const cv::Mat out8 = truncToU8(pano); // clip + 向零截断，与 numpy astype(uint8) 一致
    if (!cv::imwrite(out_path, out8)) throw std::runtime_error("cannot write image: " + out_path);
    const double t_save = msSince(tp0);

    // ---- 报告（英文/ASCII，避免 Windows 控制台编码乱码）----
    std::cout << "blend " << opt.blend << (opt.seam ? "+seam" : "") << " | parallax " << mode
              << " | threads " << opt.threads << "\n";
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

} // namespace

int main(int argc, char** argv) {
    try {
        return run(argc, argv);
    } catch (const std::exception& e) {
        std::cerr << "[error] " << e.what() << "\n";
    } catch (...) {
        std::cerr << "[error] unknown exception\n";
    }
    return 1;
}
