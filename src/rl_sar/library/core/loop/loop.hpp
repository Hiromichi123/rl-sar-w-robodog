/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LOOP_H
#define LOOP_H

#include <iostream>
#include <thread>
#include <chrono>
#include <functional>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <vector>
#include <sstream>
#include <iomanip>
#include <cstdint>
#include <cstdio>
#include "logger.hpp"

#ifdef __linux__
#include <pthread.h>
#include <sched.h>
#endif

class LoopFunc
{
public:
    /**
     * @param name        线程名(日志用)
     * @param period      周期(秒)
     * @param func        周期任务
     * @param bindCPU     绑定的 CPU 核号,-1 = 不绑核
     * @param rtPriority  实时优先级(SCHED_FIFO,1~99),-1 = 保持普通调度(SCHED_OTHER)
     *
     * 实时性分级(数字越大优先级越高):
     *   loop_imu 1kHz=90, CAN 收发=85, loop_control 200Hz=80,
     *   loop_rl 推理 50Hz=70;手柄等非硬实时线程使用普通调度。
     * 非 root 启动时需要进程具备 CAP_SYS_NICE(对二进制执行
     * `sudo setcap cap_sys_nice+ep <bin>`),否则 EPERM 自动降级为普通调度并告警。
     */
    LoopFunc(const std::string &name, float period, std::function<void()> func,
             int bindCPU = -1, int rtPriority = -1)
        : _name(name), _period(period), _func(func),
          _bindCPU(bindCPU), _rtPriority(rtPriority), _running(false) {}

    void start()
    {
        _running = true;
        std::cout << LOGGER::INFO << "[Loop] Loop start - name: " << _name << ", period: " << formatPeriod() << "ms"
                  << (_bindCPU != -1 ? ", cpu: " + std::to_string(_bindCPU) : ", cpu: unspecified")
                  << (_rtPriority > 0 ? ", rt_prio: " + std::to_string(_rtPriority) : ", rt_prio: normal")
                  << std::endl;
        _thread = std::thread(&LoopFunc::loop, this);
        // 绑核在创建线程后由外部设置(沿用原行为);实时优先级在 loop() 开头由线程自行设置,
        // 自设置只需进程具备 CAP_SYS_NICE,不依赖外部持有线程句柄。
        if (_bindCPU != -1)
        {
            setThreadAffinity(_thread.native_handle(), _bindCPU);
        }
        // 不再 detach，让 shutdown() 能正确 join 等待线程退出
    }

    void shutdown()
    {
        {
            std::unique_lock<std::mutex> lock(_mutex);
            _running = false;
            _cv.notify_one();
        }
        if (_thread.joinable())
        {
            _thread.join();
        }
        std::cout << LOGGER::INFO << "[Loop] Loop end - name: " << _name << std::endl;
    }

    /**
     * 累计超期次数(周期任务执行到点未完成,被迫追拍)。
     * 供外部(Web 遥测/健康检查)读取;正常情况下恒为 0。
     */
    uint64_t overrunCount() const { return _total_overruns.load(std::memory_order_relaxed); }

private:
    std::string _name;
    float _period;
    std::function<void()> _func;
    int _bindCPU;
    int _rtPriority;
    std::atomic<bool> _running;
    std::atomic<uint64_t> _total_overruns{0};
    std::mutex _mutex;
    std::condition_variable _cv;
    std::thread _thread;

    /**
     * 线程自行提升为 SCHED_FIFO 实时调度。
     * EPERM(无 CAP_SYS_NICE/非 root)时仅告警一次并保持普通调度,不影响运行。
     */
    void applyRealtimePriority()
    {
#ifdef __linux__
        if (_rtPriority <= 0) return;
        struct sched_param sp{};
        sp.sched_priority = _rtPriority;
        if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0)
        {
            static std::once_flag warned;
            std::call_once(warned, []() {
                std::cout << LOGGER::WARNING
                          << "[Loop] 无法设置 SCHED_FIFO 实时优先级(EPERM):以普通调度运行。"
                             "如需实时性,请执行: sudo setcap cap_sys_nice+ep <rl_real_JXG 路径>"
                          << std::endl;
            });
        }
#endif
    }

    void loop()
    {
        applyRealtimePriority();
#ifdef __linux__
        // 线程自绑核:必须在 applyRealtimePriority 之后执行。
        // 原因:start() 中由主线程绑核,但此时子线程尚未设 FIFO 优先级,
        // 亲和性巡检线程会把"非 RT 但绑大核"的线程纠正到小核 0-3。
        // 等线程自身设为 FIFO 后再绑核,巡检因 policy==FIFO 跳过,亲和性才不会被覆盖。
        if (_bindCPU != -1)
        {
            cpu_set_t cpuset;
            CPU_ZERO(&cpuset);
            CPU_SET(_bindCPU, &cpuset);
            if (pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset) != 0)
            {
                std::cout << LOGGER::WARNING << "[Loop] " << _name
                          << " 自绑核 CPU" << _bindCPU << " 失败(errno=" << errno << ")" << std::endl;
            }
        }
        // 栈预触: mlockall 只锁已物理分配的页,线程栈按需增长,深层调用时仍可能缺页。
        // 在进入实时循环前触碰 256KB 栈区(覆盖控制环调用深度),配合 MCL_FUTURE 锁死这些页。
        if (_rtPriority > 0)
        {
            constexpr int kStackPrefaultBytes = 256 * 1024;
            volatile unsigned char stack_prefault[kStackPrefaultBytes];
            for (int i = 0; i < kStackPrefaultBytes; i += 4096)
                stack_prefault[i] = static_cast<unsigned char>(i);
        }
#endif
        // Use microsecond-precision absolute deadlines to avoid millisecond
        // truncation jitter (duration_cast<ms> rounds sub-ms exec time to 0,
        // causing cycle period to oscillate ~5.0-6.0ms ≈ 167-200Hz).
        const auto period_us = std::chrono::microseconds(
            static_cast<int64_t>(_period * 1000000));
        auto next_deadline = std::chrono::steady_clock::now();

        // 超期统计(仅线程私有普通变量,健康路径零开销):
        // 每秒一个报告窗口,按严重度分级打印,健康时完全静默:
        //  - 显著超期(满足任一)→ 立即打印,同一告警间隔至少 10s 防持续过载刷屏:
        //      a) 最长连击 >= 3(连续追拍,真实调度饥饿);
        //      b) 单帧超期 >= max(50% 周期, 1ms):1kHz 环 50% 仅 0.5ms,零星亚毫秒
        //         调度抖动也触发打印偏吵,加 1ms 绝对下限过滤;200Hz 环 50%=2.5ms
        //         不受影响;超过 1ms 的真超期(如 1kHz 环迟 2 帧)仍立即报告;
        //      c) 窗口内超期帧占比 >= 10%(持续性迟到,周期预算不够)。
        //    注意不能只用"单帧幅度占周期百分比"判定——1kHz 环 0.4ms(40%)是正常
        //    唤醒抖动,而 200Hz 环 0.7ms(14%)同理;占比+连击的组合才能跨周期通用。
        //  - 轻微超期:低于上述门限 → 静默累计,每 60s 汇总一行。
        uint64_t total_overruns = 0;
        uint64_t win_overruns = 0;
        uint64_t win_cycles = 0;                                          // 窗口内完成的总周期数(算占比)
        int64_t  win_max_overrun_ns = 0;
        int      consecutive = 0, max_consecutive = 0;
        auto report_window_start = next_deadline;
        const int64_t period_ns = period_us.count() * 1000;
        constexpr int64_t kMinWarnIntervalNs = 10LL * 1000000000;         // 显著告警最小间隔 10s
        constexpr int64_t kBenignSummaryNs = 60LL * 1000000000;           // 轻微抖动汇总周期 60s
        auto last_warn_time = report_window_start;
        uint64_t benign_accum = 0;
        int64_t  benign_max_ns = 0;
        auto benign_start = report_window_start;

        while (_running)
        {
            next_deadline += period_us;
            _func();

            auto now = std::chrono::steady_clock::now();
            ++win_cycles;
            bool this_cycle_overrun = false;
            if (next_deadline > now)
            {
                std::unique_lock<std::mutex> lock(_mutex);
                if (_cv.wait_until(lock, next_deadline, [this]
                                  { return !_running; }))
                {
                    break;
                }
            }
            else
            {
                // Behind schedule: 记录超期(超出 deadline 的时长),再重置 deadline 防止无限追拍
                this_cycle_overrun = true;
                const int64_t over_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                            now - next_deadline).count();
                ++total_overruns;
                ++win_overruns;
                if (over_ns > win_max_overrun_ns) win_max_overrun_ns = over_ns;
                ++consecutive;
                if (consecutive > max_consecutive) max_consecutive = consecutive;
                _total_overruns.store(total_overruns, std::memory_order_relaxed);
                next_deadline = std::chrono::steady_clock::now();
            }
            if (!this_cycle_overrun) consecutive = 0; // 本周期按时完成,连续超期计数中断

            // 每秒汇总:按严重度分级,健康状态完全静默
            if (now - report_window_start >= std::chrono::seconds(1))
            {
                // 显著超期判定:连击>=3 / 单帧>=max(50%周期,1ms) / 占比>=10%
                constexpr int64_t kSingleFrameMinNs = 1000000;  // 单帧门限绝对下限 1ms
                const bool notable = (max_consecutive >= 3) ||
                                     (win_max_overrun_ns * 2 >= period_ns &&
                                      win_max_overrun_ns >= kSingleFrameMinNs) ||
                                     (win_cycles > 0 && win_overruns * 10 >= win_cycles);
                if (notable)
                {
                    if (now - last_warn_time >= std::chrono::nanoseconds(kMinWarnIntervalNs))
                    {
                        printf("[Loop] OVERRUN name=%s period=%ldus 本秒超期=%llu 累计=%llu"
                               " 最大超期=%.1fms 最长连击=%d%s\n",
                               _name.c_str(),
                               static_cast<int64_t>(_period * 1000000),
                               static_cast<unsigned long long>(win_overruns),
                               static_cast<unsigned long long>(total_overruns),
                               win_max_overrun_ns / 1e6,
                               max_consecutive,
                               consecutive >= 3 ? " [连续超期!检查该线程负载/同核高优先级线程]" : "");
                        last_warn_time = now;
                    }
                    benign_accum = 0;
                    benign_max_ns = 0;
                    benign_start = now;
                }
                else if (win_overruns > 0)
                {
                    benign_accum += win_overruns;
                    if (win_max_overrun_ns > benign_max_ns) benign_max_ns = win_max_overrun_ns;
                    if (now - benign_start >= std::chrono::nanoseconds(kBenignSummaryNs))
                    {
                        printf("[Loop] OVERRUN name=%s period=%ldus 近60s轻微超期%llu次"
                               " 最大=%.1fms 累计=%llu——零星调度抖动,正常范围\n",
                               _name.c_str(),
                               static_cast<int64_t>(_period * 1000000),
                               static_cast<unsigned long long>(benign_accum),
                               benign_max_ns / 1e6,
                               static_cast<unsigned long long>(total_overruns));
                        benign_accum = 0;
                        benign_max_ns = 0;
                        benign_start = now;
                    }
                }
                win_overruns = 0;
                win_cycles = 0;
                win_max_overrun_ns = 0;
                max_consecutive = 0;
                report_window_start = now;
            }
        }
    }

    std::string formatPeriod() const
    {
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(0) << _period * 1000;
        return stream.str();
    }

    void setThreadAffinity(std::thread::native_handle_type threadHandle, int cpuId)
    {
#ifdef __linux__
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(cpuId, &cpuset);
        if (pthread_setaffinity_np(threadHandle, sizeof(cpu_set_t), &cpuset) != 0)
        {
            std::ostringstream oss;
            oss << "Error setting thread affinity: CPU " << cpuId << " may not be valid or accessible.";
            throw std::runtime_error(oss.str());
        }
#else
        // Thread affinity not supported on this platform
        std::cout << LOGGER::WARNING << "Thread affinity not supported on this platform" << std::endl;
#endif
    }
};

#endif // LOOP_H
