#include "paths.hpp"

#ifdef _WIN32
#include <windows.h>
#endif

#include <string>
#include <vector>

namespace pan {

std::filesystem::path exeDir() {
#ifdef _WIN32
    std::vector<wchar_t> buf(MAX_PATH);
    for (;;) {
        DWORD len = GetModuleFileNameW(nullptr, buf.data(), static_cast<DWORD>(buf.size()));
        if (len == 0) break;
        if (len < buf.size())
            return std::filesystem::path(std::wstring(buf.data(), len)).parent_path();
        buf.resize(buf.size() * 2);
    }
#endif
    return std::filesystem::current_path();
}

std::filesystem::path findAppRoot() {
    std::filesystem::path p = exeDir();
    while (!p.empty()) {
        std::error_code ec;
        if (std::filesystem::exists(p / "config.json", ec)) return p;
        std::filesystem::path parent = p.parent_path();
        if (parent == p) break;
        p = parent;
    }
    return std::filesystem::current_path();
}

std::filesystem::path resolvePath(const std::filesystem::path& base,
                                  const std::filesystem::path& path) {
    if (path.empty()) return path;
    if (path.is_absolute()) return path.lexically_normal();
    return (base / path).lexically_normal();
}

}
