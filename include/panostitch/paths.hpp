// paths.hpp —— 可执行文件目录 / 工程根目录 / 相对路径解析
#pragma once

#include <filesystem>

namespace pan {

// 可执行文件所在目录；无法获取时回落到当前工作目录。
std::filesystem::path exeDir();

// 从可执行文件目录（其次当前工作目录）向上查找包含 config.json 的目录。
std::filesystem::path findAppRoot();

// path 为绝对路径时原样规范化返回；否则相对于 base 解析。
std::filesystem::path resolvePath(const std::filesystem::path& base,
                                  const std::filesystem::path& path);

} // namespace pan
