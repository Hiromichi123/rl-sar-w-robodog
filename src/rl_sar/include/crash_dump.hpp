#pragma once
// crash_dump.hpp — 进程级崩溃转储(仅调试用)
//
// 背景:root systemd 服务里 stdout/stderr 被 journald 以二进制行记录("[xxxB blob data]"),
// SIGABRT 的真正原因(glibc malloc 诊断 / 未捕获异常 / 段错误)被掩盖。
// 本工具把崩溃原因 + backtrace 写到世界可读的固定文件 /tmp/rl_crash.log(0666),
// 便于普通用户事后用 addr2line 解码地址定位源码。
//
// 用法:main() 第一行 InstallCrashDumper();

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <unistd.h>
#include <execinfo.h>
#include <fcntl.h>
#include <exception>
#include <typeinfo>
#include <string>

namespace crash_dump_detail
{

inline const char *kCrashPath = "/tmp/rl_crash.log";

// async-signal-safe 写入
inline void wr(const char *s)
{
    if (s) ::write(STDERR_FILENO, s, std::strlen(s));
}

inline void wr_num(long v)
{
    char b[32];
    snprintf(b, sizeof(b), "%ld", v);
    wr(b);
}

inline void dump_backtrace(int fd, void *ctx_ucontext)
{
    (void)ctx_ucontext;
    void *frames[64];
    int n = backtrace(frames, 64);
    dprintf(fd, "--- backtrace (%d frames) ---\n", n);
    backtrace_symbols_fd(frames, n, fd);
}

inline void sig_handler(int sig, siginfo_t *info, void *ucontext)
{
    int fd = ::open(kCrashPath, O_WRONLY | O_CREAT | O_APPEND, 0666);
    if (fd >= 0)
    {
        time_t t = ::time(nullptr);
        char tbuf[32];
        std::strftime(tbuf, sizeof(tbuf), "%H:%M:%S", std::localtime(&t));
        dprintf(fd, "\n===== CRASH %s pid=%ld signal=%d (%s) =====\n",
                tbuf, (long)getpid(), sig, strsignal(sig));
        if (info)
        {
            dprintf(fd, "si_code=%d fault_addr=%p\n", info->si_code, info->si_addr);
        }
        dump_backtrace(fd, ucontext);
        ::fsync(fd);
        ::close(fd);
        ::chmod(kCrashPath, 0666);
    }
    // 同时打到 stderr(journald),即便被标记为 blob 也尽力保留
    wr("\n[crash_dump] FATAL signal="); wr_num(sig); wr("\n");

    // 恢复默认处置并重新抛出,保留 core/systemd 的原始记录
    ::signal(sig, SIG_DFL);
    ::raise(sig);
}

inline void terminate_handler()
{
    std::string msg = "unknown";
    try
    {
        std::rethrow_exception(std::current_exception());
    }
    catch (const std::exception &e)
    {
        msg = std::string(typeid(e).name()) + ": " + e.what();
    }
    catch (...)
    {
        msg = "non-std exception";
    }

    int fd = ::open(kCrashPath, O_WRONLY | O_CREAT | O_APPEND, 0666);
    if (fd >= 0)
    {
        time_t t = ::time(nullptr);
        char tbuf[32];
        std::strftime(tbuf, sizeof(tbuf), "%H:%M:%S", std::localtime(&t));
        dprintf(fd, "\n===== TERMINATE %s pid=%ld =====\nuncaught: %s\n",
                tbuf, (long)getpid(), msg.c_str());
        void *frames[64];
        int n = backtrace(frames, 64);
        dprintf(fd, "--- backtrace (%d frames) ---\n", n);
        backtrace_symbols_fd(frames, n, fd);
        ::fsync(fd);
        ::close(fd);
        ::chmod(kCrashPath, 0666);
    }
    fprintf(stderr, "\n[crash_dump] terminate: %s\n", msg.c_str());

    std::_Exit(134);
}

} // namespace crash_dump_detail

inline void InstallCrashDumper()
{
    std::set_terminate(crash_dump_detail::terminate_handler);
    struct sigaction sa {};
    sa.sa_sigaction = crash_dump_detail::sig_handler;
    sa.sa_flags = SA_SIGINFO | SA_RESETHAND;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGABRT, &sa, nullptr);
    sigaction(SIGSEGV, &sa, nullptr);
    sigaction(SIGBUS, &sa, nullptr);
    sigaction(SIGFPE, &sa, nullptr);
    sigaction(SIGILL, &sa, nullptr);
}
