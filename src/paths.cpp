// paths.cpp —— 可执行文件目录与工程根目录定位
#include <panostitch/paths.hpp>

#if defined(_WIN32)
#include <windows.h>
#elif defined(__linux__)
#include <unistd.h>
#elif defined(__APPLE__)
#include <mach-o/dyld.h>
#endif

#include <array>
#include <cstdint>
#include <string>
#include <system_error>
#include <vector>

namespace fs = std::filesystem;

namespace pan {

namespace {

// 从 start 开始逐级向上查找包含 config.json 的目录；找不到返回空路径。
fs::path findUpwards(fs::path start) {
    while (!start.empty()) {
        std::error_code ec;
        if (fs::exists(start / "config.json", ec)) return start;
        const fs::path parent = start.parent_path();
        if (parent == start) break;
        start = parent;
    }
    return {};
}

} // namespace

fs::path exeDir() {
#if defined(_WIN32)
    std::vector<wchar_t> buf(MAX_PATH);
    for (;;) {
        const DWORD len = GetModuleFileNameW(nullptr, buf.data(), static_cast<DWORD>(buf.size()));
        if (len == 0) break;
        if (len < buf.size()) {
            return fs::path(std::wstring(buf.data(), len)).parent_path();
        }
        buf.resize(buf.size() * 2);
    }
#elif defined(__linux__)
    std::array<char, 4096> buf{};
    const ssize_t len = ::readlink("/proc/self/exe", buf.data(), buf.size() - 1);
    if (len > 0) {
        return fs::path(std::string(buf.data(), static_cast<size_t>(len))).parent_path();
    }
#elif defined(__APPLE__)
    std::vector<char> buf(4096);
    uint32_t size = static_cast<uint32_t>(buf.size());
    if (::_NSGetExecutablePath(buf.data(), &size) != 0) {
        buf.resize(size);
        size = static_cast<uint32_t>(buf.size());
    }
    if (::_NSGetExecutablePath(buf.data(), &size) == 0) {
        return fs::path(buf.data()).parent_path();
    }
#endif
    return fs::current_path();
}

fs::path findAppRoot() {
    // 优先从可执行文件所在目录向上找（安装后也能定位随包数据）；
    // 找不到时再从当前工作目录向上找（开发时从任意子目录启动）。
    const fs::path exe_root = findUpwards(exeDir());
    if (!exe_root.empty()) return exe_root;
    const fs::path cwd_root = findUpwards(fs::current_path());
    if (!cwd_root.empty()) return cwd_root;
    return fs::current_path();
}

fs::path resolvePath(const fs::path& base, const fs::path& path) {
    if (path.empty()) return path;
    if (path.is_absolute()) return path.lexically_normal();
    return (base / path).lexically_normal();
}

} // namespace pan
