#pragma once
#include <atomic>
#include <cstdint>
#include <mutex>
#include <thread>

namespace oboe { class AudioStream; }

namespace dedective {

// Thread-safe audio output.
// Uses Oboe on Android, PulseAudio elsewhere.
// Accepts 8 kHz 16-bit mono PCM from the DECT voice decoder and plays it.
class AudioOutput {
public:
    AudioOutput();
    ~AudioOutput();

    bool start();
    void stop();

    // Thread-safe: enqueue decoded PCM samples for playback
    void write_samples(const int16_t* samples, size_t count);

    void  set_muted(bool m);
    bool  is_muted()   const { return muted_.load(std::memory_order_relaxed); }

    void  set_volume(float v);   // 0.0 – 1.0
    float volume()     const { return volume_.load(std::memory_order_relaxed); }

    bool  is_running() const { return running_.load(std::memory_order_relaxed); }

private:
    // Power-of-2 ring buffer (~4 s at 8 kHz)
    static constexpr size_t RING_SIZE = 32768;
    static constexpr size_t RING_MASK = RING_SIZE - 1;

    int16_t ring_[RING_SIZE]{};
    std::atomic<size_t> read_pos_{0};
    std::atomic<size_t> write_pos_{0};

#if defined(__ANDROID__)
    oboe::AudioStream* stream_ = nullptr;
    int16_t last_sample_ = 0; // for smooth fade on underrun
    friend class AudioDataCallback;
#else
    void* pa_handle_ = nullptr;   // pa_simple*
    std::thread thread_;
    void audio_loop();
#endif

    std::atomic<bool>  running_{false};
    std::atomic<bool>  muted_{false};
    std::atomic<float> volume_{0.8f};
};

} // namespace dedective
