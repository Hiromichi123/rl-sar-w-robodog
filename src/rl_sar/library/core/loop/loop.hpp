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
        // 每秒一个报告窗口,窗口内出现超期才打印一行,健康时完全静默。
        uint64_t total_overruns = 0;
        uint64_t win_overruns = 0;
        int64_t  win_max_overrun_ns = 0;
        int      consecutive = 0, max_consecutive = 0;
        auto report_window_start = next_deadline;

        while (_running)
        {
            next_deadline += period_us;
            _func();

            auto now = std::chrono::steady_clock::now();
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

            // 每秒汇总:仅在本窗口发生过超期时打印,避免健康状态下刷屏
            if (now - report_window_start >= std::chrono::seconds(1))
            {
                if (win_overruns > 0)
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
                }
                win_overruns = 0;
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
