// parallel.hpp —— 极简并行 for（工作窃取式调度）
//
// 只用于**任务彼此独立、写不同输出**的循环（如按相机/按层/按接缝对），
// 因此结果与单线程**逐位一致**（不改变浮点累加顺序）。
#pragma once

#include <algorithm>
#include <atomic>
#include <thread>
#include <vector>

namespace pan {

// 对 i∈[0,n) 执行 f(i)；threads<=1 时串行。
template <class F>
void parallel_for(int n, int threads, F&& f) {
    if (threads <= 1 || n <= 1) {
        for (int i = 0; i < n; ++i) f(i);
        return;
    }
    const int T = std::min(threads, n);
    std::atomic<int> next{0};
    std::vector<std::thread> pool;
    pool.reserve(T);
    for (int t = 0; t < T; ++t) {
        pool.emplace_back([&]() {
            int i;
            while ((i = next.fetch_add(1)) < n) f(i);
        });
    }
    for (auto& th : pool) th.join();
}

// 把 [0,n) 切成 T 段，返回第 t 段的 [begin,end)
inline void block_range(int n, int t, int T, int& begin, int& end) {
    begin = (int)((long long)n * t / T);
    end = (int)((long long)n * (t + 1) / T);
}

} // namespace pan
