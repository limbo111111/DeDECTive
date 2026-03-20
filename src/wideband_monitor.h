#pragma once

#include "dect_channels.h"
#include "packet_decoder.h"
#include "packet_receiver.h"
#include "phase_diff.h"
#include "audio_output.h"
#include "dc_blocker.h"

#include <array>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

namespace dedective {

inline constexpr uint32_t WIDEBAND_SAMPLE_RATE = 18'432'000;
inline constexpr uint64_t WIDEBAND_CENTER_FREQ_HZ =
    (US_DECT_CHANNELS.front().freq_hz + US_DECT_CHANNELS.back().freq_hz) / 2;

struct WidebandChannelView {
    int      channel_number      = -1;
    uint64_t freq_hz             = 0;
    float    smoothed_power_db   = 0.0f;
    float    relative_power_db   = 0.0f;
    bool     active              = false;
    bool     voice_detected      = false;
    bool     qt_synced           = false;
    int      active_parts        = 0;
    uint64_t packets_seen        = 0;
    uint64_t voice_frames_ok     = 0;
    uint64_t voice_xcrc_fail     = 0;
    uint64_t voice_skipped       = 0;
};

struct WidebandSnapshot {
    bool ready = false;
    size_t buffered_samples = 0;
    float noise_floor_db = 0.0f;
    std::vector<float> fft_db;
    std::vector<float> waterfall_db;
    size_t waterfall_rows = 0;
    size_t waterfall_cols = 0;
    std::array<WidebandChannelView, US_DECT_CHANNELS.size()> channels{};
};

class WidebandMonitor {
public:
    static constexpr size_t FFT_SIZE = 16'384;
    static constexpr size_t FFT_PLOT_BINS = 256;
    static constexpr size_t WATERFALL_HISTORY = 160;

    WidebandMonitor();

    void ingest(const std::complex<float>* samples, size_t n);
    bool update_visuals();
    WidebandSnapshot snapshot() const;
    void render_frame();

    // Audio output control
    void set_audio_output(AudioOutput* audio);
    void set_audio_channel(int channel_index);  // -1 = auto (first voice channel)
    int  audio_channel() const;

    // DC offset correction
    void set_dc_block(bool enabled);
    bool dc_block_enabled() const;

private:
    struct ChannelDecoderSnapshot {
        bool     voice_detected = false;
        bool     qt_synced      = false;
        int      active_parts   = 0;
        uint64_t packets_seen   = 0;
        uint64_t voice_frames_ok  = 0;
        uint64_t voice_xcrc_fail  = 0;
        uint64_t voice_skipped    = 0;
    };

    std::vector<std::complex<float>> ring_buffer_;
    std::array<float, US_DECT_CHANNELS.size()> smoothed_power_db_;
    std::array<float, US_DECT_CHANNELS.size()> peak_delta_db_;
    std::array<PhaseDiff, US_DECT_CHANNELS.size()> phase_diff_;
    std::array<std::complex<float>, US_DECT_CHANNELS.size()> mixer_;
    std::array<std::complex<float>, US_DECT_CHANNELS.size()> mixer_step_;
    std::array<std::complex<float>, US_DECT_CHANNELS.size()> decim_accum_;
    std::array<int, US_DECT_CHANNELS.size()> decim_phase_;
    std::array<std::unique_ptr<PacketReceiver>, US_DECT_CHANNELS.size()> receivers_;
    std::array<std::unique_ptr<PacketDecoder>, US_DECT_CHANNELS.size()> decoders_;
    std::array<bool, US_DECT_CHANNELS.size()> voice_detected_;
    std::array<int, US_DECT_CHANNELS.size()> active_parts_;
    std::array<uint64_t, US_DECT_CHANNELS.size()> packets_seen_;
    std::array<float, FFT_PLOT_BINS> fft_plot_db_;
    std::vector<float> waterfall_history_;
    std::array<WidebandChannelView, US_DECT_CHANNELS.size()> channel_views_;
    size_t write_pos_;
    size_t buffered_samples_;
    uint64_t sample_counter_;
    bool have_history_;
    bool visual_ready_;
    size_t waterfall_head_;
    size_t waterfall_rows_filled_;
    float noise_floor_db_;
    mutable std::mutex ring_mutex_;
    mutable std::mutex status_mutex_;
    mutable std::mutex visual_mutex_;

    // Audio routing
    AudioOutput* audio_out_ = nullptr;
    int audio_channel_ = -1;  // -1 = auto
    mutable std::mutex audio_mutex_;

    // DC offset correction
    DCBlocker dc_blocker_;
    bool dc_block_enabled_ = true;
    std::vector<std::complex<float>> dc_buf_;  // reusable scratch for filtered samples

    bool snapshot_latest_block(std::vector<std::complex<float>>& out, size_t& buffered_samples);
    void snapshot_channel_state(
        std::array<ChannelDecoderSnapshot, US_DECT_CHANNELS.size()>& out) const;
    void render_waiting_screen(size_t buffered_samples) const;
    void process_sample(std::complex<float> sample) noexcept;
    void on_channel_update(size_t channel_index, const PartInfo parts[], int count) noexcept;
    void on_voice_packet(size_t channel_index, int rx_id,
                         const int16_t* pcm, size_t count) noexcept;

    static void fft_inplace(std::vector<std::complex<float>>& data);
};

} // namespace dedective
