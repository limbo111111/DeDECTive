#include "audio_output.h"

#if defined(__ANDROID__)
#include <oboe/Oboe.h>
#else
#include <pulse/simple.h>
#include <pulse/error.h>
#endif

#include <algorithm>
#include <cstring>
#include <cstdio>

namespace dedective {

AudioOutput::AudioOutput() = default;

AudioOutput::~AudioOutput() { stop(); }

#if defined(__ANDROID__)

class AudioDataCallback : public oboe::AudioStreamDataCallback {
public:
    explicit AudioDataCallback(AudioOutput* output) : output_(output) {}

    oboe::DataCallbackResult onAudioReady(oboe::AudioStream* /*oboeStream*/,
                                          void* audioData,
                                          int32_t numFrames) override {
        if (!output_->running_.load(std::memory_order_relaxed)) {
            std::memset(audioData, 0, static_cast<size_t>(numFrames) * sizeof(int16_t));
            return oboe::DataCallbackResult::Continue;
        }

        auto* buf = static_cast<int16_t*>(audioData);
        size_t rp = output_->read_pos_.load(std::memory_order_relaxed);
        size_t wp = output_->write_pos_.load(std::memory_order_acquire);
        size_t avail = wp - rp;

        float vol = output_->muted_.load(std::memory_order_relaxed)
                  ? 0.0f
                  : output_->volume_.load(std::memory_order_relaxed);

        size_t frames_to_read = std::min(avail, static_cast<size_t>(numFrames));

        for (size_t i = 0; i < frames_to_read; ++i) {
            int32_t s = output_->ring_[(rp + i) & AudioOutput::RING_MASK];
            buf[i] = static_cast<int16_t>(std::clamp(
                static_cast<int32_t>(s * vol), -32768, 32767));
        }

        if (frames_to_read > 0) {
            output_->last_sample_ = buf[frames_to_read - 1];
            output_->read_pos_.store(rp + frames_to_read, std::memory_order_release);
        }

        // Fill remaining requested frames (underrun) with a fade out to silence
        if (frames_to_read < static_cast<size_t>(numFrames)) {
            size_t remaining = static_cast<size_t>(numFrames) - frames_to_read;
            for (size_t i = 0; i < remaining; ++i) {
                float fade = 1.0f - static_cast<float>(i) / static_cast<float>(remaining);
                buf[frames_to_read + i] = static_cast<int16_t>(output_->last_sample_ * fade);
            }
            output_->last_sample_ = 0;
        }

        return oboe::DataCallbackResult::Continue;
    }

private:
    AudioOutput* output_;
};

bool AudioOutput::start() {
    if (running_.load()) return true;

    oboe::AudioStreamBuilder builder;
    builder.setDirection(oboe::Direction::Output)
           ->setPerformanceMode(oboe::PerformanceMode::LowLatency)
           ->setSharingMode(oboe::SharingMode::Exclusive)
           ->setFormat(oboe::AudioFormat::I16)
           ->setChannelCount(oboe::ChannelCount::Mono)
           ->setSampleRate(8000)
           ->setDataCallback(new AudioDataCallback(this));

    oboe::Result result = builder.openStream(&stream_);
    if (result != oboe::Result::OK) {
        std::fprintf(stderr, "Failed to create Oboe audio stream: %s\n", oboe::convertToText(result));
        return false;
    }

    result = stream_->requestStart();
    if (result != oboe::Result::OK) {
        std::fprintf(stderr, "Failed to start Oboe audio stream: %s\n", oboe::convertToText(result));
        stream_->close();
        delete stream_->getDataCallback();
        stream_ = nullptr;
        return false;
    }

    read_pos_  = 0;
    write_pos_ = 0;
    running_   = true;
    return true;
}

void AudioOutput::stop() {
    running_ = false;
    if (stream_) {
        stream_->requestStop();
        stream_->close();
        delete stream_->getDataCallback();
        stream_ = nullptr;
    }
}

#else

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

#endif

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

#if !defined(__ANDROID__)
void AudioOutput::audio_loop() {
    constexpr size_t FRAME = 80;  // one DECT voice frame
    int16_t buf[FRAME];
    int16_t last_sample = 0;      // for smooth fade on underrun

    while (running_.load(std::memory_order_relaxed)) {
        size_t rp = read_pos_.load(std::memory_order_relaxed);
        size_t wp = write_pos_.load(std::memory_order_acquire);
        size_t avail = wp - rp;

        if (avail < FRAME) {
            // Underrun — fade from last sample to silence over the frame
            // to avoid a hard discontinuity (click/pop).
            for (size_t i = 0; i < FRAME; ++i) {
                float fade = 1.0f - static_cast<float>(i) / static_cast<float>(FRAME);
                buf[i] = static_cast<int16_t>(last_sample * fade);
            }
            last_sample = 0;
        } else {
            float vol = muted_.load(std::memory_order_relaxed)
                      ? 0.0f
                      : volume_.load(std::memory_order_relaxed);
            for (size_t i = 0; i < FRAME; ++i) {
                int32_t s = ring_[(rp + i) & RING_MASK];
                buf[i] = static_cast<int16_t>(std::clamp(
                    static_cast<int32_t>(s * vol), -32768, 32767));
            }
            last_sample = buf[FRAME - 1];
            read_pos_.store(rp + FRAME, std::memory_order_release);
        }

        int error = 0;
        pa_simple_write(static_cast<pa_simple*>(pa_handle_),
                        buf, sizeof(buf), &error);
    }
}
#endif

} // namespace dedective
