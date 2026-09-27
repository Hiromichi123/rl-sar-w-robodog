#include "briefing_audio_player.hpp"

#include <alsa/asoundlib.h>

#include <algorithm>
#include <cstring>
#include <filesystem>
#include <fstream>

namespace fs = std::filesystem;

namespace rl_briefing {

namespace {

struct WavData {
    int sample_rate = 0;
    int channels = 0;
    std::vector<int16_t> mono;
};

// 极简 WAV(PCM16)解析，抽出单声道样本（多声道取第一声道）。
bool load_wav_mono(const std::string& path, WavData& out) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;

    auto read_u16 = [&f]() -> uint16_t {
        uint16_t v = 0;
        f.read(reinterpret_cast<char*>(&v), 2);
        return v;
    };
    auto read_u32 = [&f]() -> uint32_t {
        uint32_t v = 0;
        f.read(reinterpret_cast<char*>(&v), 4);
        return v;
    };

    char riff[4];
    f.read(riff, 4);
    if (std::string(riff, 4) != "RIFF") return false;
    read_u32();
    char wave[4];
    f.read(wave, 4);
    if (std::string(wave, 4) != "WAVE") return false;

    uint16_t fmt_channels = 1;
    uint32_t fmt_sr = 48000;
    uint16_t bits = 16;
    std::vector<char> data;

    while (f) {
        char id[4];
        f.read(id, 4);
        if (!f) break;
        uint32_t size = read_u32();
        std::string cid(id, 4);
        if (cid == "fmt ") {
            read_u16();
            fmt_channels = read_u16();
            fmt_sr = read_u32();
            read_u32();
            read_u16();
            bits = read_u16();
        } else if (cid == "data") {
            data.resize(size);
            f.read(data.data(), static_cast<std::streamsize>(size));
        } else {
            f.seekg(static_cast<std::streamoff>(size), std::ios::cur);
        }
    }
    if (bits != 16) return false;

    size_t n = data.size() / 2;
    std::vector<int16_t> samples(n);
    std::memcpy(samples.data(), data.data(), n * 2);

    if (fmt_channels <= 1) {
        out.mono = std::move(samples);
    } else {
        out.mono.reserve(n / fmt_channels);
        for (size_t i = 0; i < n; i += fmt_channels) {
            out.mono.push_back(samples[i]);
        }
    }
    out.sample_rate = static_cast<int>(fmt_sr);
    out.channels = fmt_channels;
    return !out.mono.empty();
}

// 线性插值重采样（单声道 int16）。
std::vector<int16_t> resample_mono(const std::vector<int16_t>& in, int src_sr,
                                   int dst_sr) {
    if (src_sr == dst_sr || in.empty()) return in;
    double ratio = static_cast<double>(dst_sr) / src_sr;
    size_t n_out = static_cast<size_t>(in.size() * ratio);
    std::vector<int16_t> out(n_out);
    for (size_t i = 0; i < n_out; ++i) {
        double pos = i / ratio;
        size_t i0 = static_cast<size_t>(pos);
        size_t i1 = std::min(i0 + 1, in.size() - 1);
        double frac = pos - static_cast<double>(i0);
        double v = in[i0] * (1.0 - frac) + in[i1] * frac;
        out[i] = static_cast<int16_t>(v);
    }
    return out;
}

}  // namespace

BriefingAudioPlayer::BriefingAudioPlayer(std::string audio_dir,
                                         std::string device, int sample_rate,
                                         int channels)
    : audio_dir_(std::move(audio_dir)),
      device_(std::move(device)),
      sample_rate_(sample_rate),
      channels_(channels) {}

BriefingAudioPlayer::~BriefingAudioPlayer() {
    stop_ = true;
    stop();
    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (worker_.joinable()) worker_.join();
    }
    close_device();
}

bool BriefingAudioPlayer::play(const std::string& name) {
    // 拆逗号分隔的多个场景名，并去首尾空白 + 去 .wav 后缀
    std::vector<std::string> names;
    std::string cur;
    auto trim = [](std::string s) {
        while (!s.empty() && (s.back() == ' ' || s.back() == '\t' ||
                              s.back() == '\n' || s.back() == '\r')) {
            s.pop_back();
        }
        while (!s.empty() && (s.front() == ' ' || s.front() == '\t')) {
            s.erase(s.begin());
        }
        return s;
    };
    for (char c : name) {
        if (c == ',') {
            if (!cur.empty()) {
                names.push_back(trim(cur));
                cur.clear();
            }
        } else {
            cur += c;
        }
    }
    if (!cur.empty()) names.push_back(trim(cur));

    for (auto& n : names) {
        if (n.size() > 4 && n.substr(n.size() - 4) == ".wav") {
            n = n.substr(0, n.size() - 4);
        }
    }
    if (names.empty()) return false;

    // 至少一个场景存在才算有效
    bool any = false;
    for (const auto& n : names) {
        if (fs::exists(fs::path(audio_dir_) / (n + ".wav"))) {
            any = true;
            break;
        }
    }
    if (!any) return false;

    std::lock_guard<std::mutex> lk(mtx_);
    ++gen_;
    stop_.store(true);          // 打断旧播放
    if (worker_.joinable()) worker_.join();
    stop_.store(false);
    const uint64_t gen = gen_;
    worker_ = std::thread([this, names, gen]() { worker(names, gen); });
    return true;
}

void BriefingAudioPlayer::stop() { stop_.store(true); }

std::vector<std::string> BriefingAudioPlayer::list() const {
    std::vector<std::string> out;
    std::error_code ec;
    if (!fs::is_directory(audio_dir_, ec)) return out;
    for (const auto& e : fs::directory_iterator(audio_dir_)) {
        if (e.path().extension() == ".wav") {
            out.push_back(e.path().stem().string());
        }
    }
    std::sort(out.begin(), out.end());
    return out;
}

bool BriefingAudioPlayer::open_device() {
    if (pcm_) return true;
    int rc = snd_pcm_open(&pcm_, device_.c_str(), SND_PCM_STREAM_PLAYBACK, 0);
    if (rc < 0) {
        pcm_ = nullptr;
        return false;
    }
    rc = snd_pcm_set_params(pcm_, SND_PCM_FORMAT_S16_LE,
                            SND_PCM_ACCESS_RW_INTERLEAVED, channels_,
                            sample_rate_, 1, 100000);
    if (rc < 0) {
        snd_pcm_close(pcm_);
        pcm_ = nullptr;
        return false;
    }
    return true;
}

void BriefingAudioPlayer::close_device() {
    if (pcm_) {
        snd_pcm_drain(pcm_);
        snd_pcm_close(pcm_);
        pcm_ = nullptr;
    }
}

bool BriefingAudioPlayer::play_pcm(const std::vector<int16_t>& mono) {
    if (!pcm_ || mono.empty()) return false;

    const size_t period_frames = 1024;
    std::vector<int16_t> interleaved;
    if (channels_ > 1) interleaved.resize(period_frames * channels_);

    size_t pos = 0;
    const size_t total = mono.size();
    while (pos < total) {
        if (stop_.load()) {
            snd_pcm_drop(pcm_);
            return false;
        }
        size_t n = std::min(period_frames, total - pos);
        snd_pcm_sframes_t w;
        if (channels_ == 1) {
            w = snd_pcm_writei(pcm_, mono.data() + pos,
                               static_cast<snd_pcm_uframes_t>(n));
        } else {
            for (size_t i = 0; i < n; ++i) {
                for (int c = 0; c < channels_; ++c) {
                    interleaved[i * channels_ + c] = mono[pos + i];
                }
            }
            w = snd_pcm_writei(pcm_, interleaved.data(),
                               static_cast<snd_pcm_uframes_t>(n));
        }
        if (w < 0) {
            w = snd_pcm_recover(pcm_, w, 1);
            if (w < 0) return false;
            continue;
        }
        pos += static_cast<size_t>(w);
    }
    snd_pcm_drain(pcm_);
    return true;
}

void BriefingAudioPlayer::worker(std::vector<std::string> names, uint64_t gen) {
    for (const auto& name : names) {
        if (stop_.load() || gen != gen_) return;

        fs::path wav = fs::path(audio_dir_) / (name + ".wav");
        WavData wd;
        if (!load_wav_mono(wav.string(), wd)) continue;  // 跳过不存在的场景

        if (stop_.load() || gen != gen_) return;

        std::vector<int16_t> pcm =
            resample_mono(wd.mono, wd.sample_rate, sample_rate_);

        if (!open_device()) return;
        if (gen == gen_ && !stop_.load()) {
            play_pcm(pcm);
        }
        close_device();
    }
}

}  // namespace rl_briefing
