#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// ALSA 前向声明（实现在 .cpp 中链接 libasound）
extern "C" {
struct _snd_pcm;
typedef struct _snd_pcm snd_pcm_t;
}

namespace rl_briefing {

/// 场景音频播放器：非阻塞播放预设 wav 到 USB 声卡，支持打断。
/// 内部自带 ALSA 播放、wav 解析、线性重采样、播放线程。
class BriefingAudioPlayer {
public:
    BriefingAudioPlayer(std::string audio_dir, std::string device,
                        int sample_rate = 48000, int channels = 2);
    ~BriefingAudioPlayer();

    BriefingAudioPlayer(const BriefingAudioPlayer&) = delete;
    BriefingAudioPlayer& operator=(const BriefingAudioPlayer&) = delete;

    /// 非阻塞播放 <audio_dir>/<name>.wav（name 可带/不带 .wav 后缀）。
    /// 支持逗号分隔多个场景名，依次顺序播放，如 "forklift,elevator"。
    /// 所有场景名都不存在时返回 false。
    bool play(const std::string& name);

    /// 打断当前播放。
    void stop();

    /// 列出 audio_dir 下所有 .wav 文件名（不含后缀）。
    std::vector<std::string> list() const;

private:
    bool open_device();
    void close_device();
    // 播放单声道 PCM，期间可被 stop() 打断；返回 false 表示被打断/出错
    bool play_pcm(const std::vector<int16_t>& mono);
    void worker(std::vector<std::string> names, uint64_t gen);

    std::string audio_dir_;
    std::string device_;
    int sample_rate_;
    int channels_;
    snd_pcm_t* pcm_ = nullptr;

    std::mutex mtx_;
    std::thread worker_;
    std::atomic<uint64_t> gen_{0};
    std::atomic<bool> stop_{false};
};

}  // namespace rl_briefing
