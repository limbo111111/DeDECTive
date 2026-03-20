#pragma once

#include "dect_channels.h"
#include "packet_decoder.h"
#include "packet_receiver.h"
#include "phase_diff.h"

#include <array>
#include <atomic>
#include <chrono>
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

class WidebandMonitor {
public:
    WidebandMonitor();

    void ingest(const std::complex<float>* samples, size_t n);
    void render_frame();

private:
    static constexpr size_t FFT_SIZE = 16'384;

    struct ChannelSnapshot {
        bool     voice_detected = false;
        int      active_parts   = 0;
        uint64_t packets_seen   = 0;
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
    size_t write_pos_;
    size_t buffered_samples_;
    uint64_t sample_counter_;
    bool   have_history_;
    std::mutex ring_mutex_;
    std::mutex status_mutex_;

    bool snapshot_latest_block(std::vector<std::complex<float>>& out, size_t& buffered_samples);
    void snapshot_channel_state(std::array<ChannelSnapshot, US_DECT_CHANNELS.size()>& out);
    void render_waiting_screen(size_t buffered_samples) const;
    void process_sample(std::complex<float> sample) noexcept;
    void on_channel_update(size_t channel_index, const PartInfo parts[], int count) noexcept;

    static void fft_inplace(std::vector<std::complex<float>>& data);
};

} // namespace dedective
