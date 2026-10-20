#pragma once

#include <filesystem>

namespace pan {

std::filesystem::path exeDir();
std::filesystem::path findAppRoot();
std::filesystem::path resolvePath(const std::filesystem::path& base,
                                  const std::filesystem::path& path);

}
