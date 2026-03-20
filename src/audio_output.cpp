#include "audio_output.h"

#include <pulse/simple.h>
#include <pulse/error.h>

#include <algorithm>
#include <cstring>

namespace dedective {

AudioOutput::AudioOutput() = default;

AudioOutput::~AudioOutput() { stop(); }

bool AudioOutput::start() {
    if (running_.load()) return true;

    pa_sample_spec ss{};
    ss.format   = PA_SAMPLE_S16LE;
    ss.rate     = 8000;
    ss.channels = 1;

    pa_buffer_attr ba{};
    ba.maxlength = 8000 * 2;   // 1 s max buffer
    ba.tlength   = 800  * 2;   // 100 ms target latency
    ba.prebuf    = 400  * 2;   // 50 ms pre-buffer
    ba.minreq    = 80   * 2;   // minimum request (1 DECT frame)
    ba.fragsize  = (uint32_t)-1;

    int error = 0;
    pa_handle_ = pa_simple_new(nullptr, "DeDECTive", PA_STREAM_PLAYBACK,
                               nullptr, "DECT Voice", &ss, nullptr, &ba, &error);
    if (!pa_handle_) return false;

    read_pos_  = 0;
    write_pos_ = 0;
    running_   = true;
    thread_    = std::thread(&AudioOutput::audio_loop, this);
    return true;
}

void AudioOutput::stop() {
    running_ = false;
    if (thread_.joinable()) thread_.join();
    if (pa_handle_) {
        pa_simple_drain(static_cast<pa_simple*>(pa_handle_), nullptr);
        pa_simple_free(static_cast<pa_simple*>(pa_handle_));
        pa_handle_ = nullptr;
    }
}

void AudioOutput::write_samples(const int16_t* samples, size_t count) {
    if (!running_.load(std::memory_order_relaxed)) return;

    size_t wp = write_pos_.load(std::memory_order_relaxed);
    size_t rp = read_pos_.load(std::memory_order_acquire);

    // If ring would overflow, skip oldest data
    if (wp - rp + count > RING_SIZE)
        read_pos_.store(wp + count - RING_SIZE, std::memory_order_release);

    for (size_t i = 0; i < count; ++i)
        ring_[(wp + i) & RING_MASK] = samples[i];

    write_pos_.store(wp + count, std::memory_order_release);
}

void AudioOutput::set_muted(bool m)  { muted_ = m; }
void AudioOutput::set_volume(float v){ volume_ = std::clamp(v, 0.0f, 1.0f); }

void AudioOutput::audio_loop() {
    constexpr size_t FRAME = 80;  // one DECT voice frame
    int16_t buf[FRAME];

    while (running_.load(std::memory_order_relaxed)) {
        size_t rp = read_pos_.load(std::memory_order_relaxed);
        size_t wp = write_pos_.load(std::memory_order_acquire);
        size_t avail = wp - rp;

        if (avail < FRAME) {
            // Underrun — output silence to keep stream alive
            std::memset(buf, 0, sizeof(buf));
        } else {
            float vol = muted_.load(std::memory_order_relaxed)
                      ? 0.0f
                      : volume_.load(std::memory_order_relaxed);
            for (size_t i = 0; i < FRAME; ++i) {
                int32_t s = ring_[(rp + i) & RING_MASK];
                buf[i] = static_cast<int16_t>(std::clamp(
                    static_cast<int32_t>(s * vol), -32768, 32767));
            }
            read_pos_.store(rp + FRAME, std::memory_order_release);
        }

        int error = 0;
        pa_simple_write(static_cast<pa_simple*>(pa_handle_),
                        buf, sizeof(buf), &error);
    }
}

} // namespace dedective
