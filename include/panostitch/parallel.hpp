// parallel.hpp —— 极简并行 for（工作窃取式调度）
//
// 只用于**任务彼此独立、写不同输出**的循环（如按相机/按层/按接缝对），
// 因此结果与单线程**逐位一致**（不改变浮点累加顺序）。
#pragma once

#include <algorithm>
#include <atomic>
#include <exception>
#include <mutex>
#include <thread>
#include <vector>

namespace pan {

// 对 i∈[0,n) 执行 f(i)；threads<=1 时串行。
// 工作线程中的异常会在所有线程 join 后重新抛出，避免 std::terminate。
template <class F> void parallel_for(int n, int threads, F&& f) {
    if (n <= 0) return;
    if (threads <= 1 || n <= 1) {
        for (int i = 0; i < n; ++i)
            f(i);
        return;
    }

    const int worker_count = std::min(threads, n);
    std::atomic<int> next{0};
    std::exception_ptr error;
    std::mutex error_mutex;

    auto worker = [&]() {
        try {
            for (;;) {
                const int i = next.fetch_add(1);
                if (i >= n) break;
                f(i);
            }
        } catch (...) {
            std::lock_guard<std::mutex> lock(error_mutex);
            if (!error) error = std::current_exception();
        }
    };

    std::vector<std::thread> pool;
    pool.reserve(worker_count);
    for (int t = 0; t < worker_count; ++t)
        pool.emplace_back(worker);
    for (auto& thread : pool)
        thread.join();

    if (error) std::rethrow_exception(error);
}

} // namespace pan
