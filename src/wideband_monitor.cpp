#include "wideband_monitor.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace dedective {

namespace {

constexpr float PI_F = 3.14159265358979323846f;
constexpr float CHANNEL_HALF_BW_HZ = 550'000.0f;
constexpr float FFT_EPSILON = 1e-12f;

size_t fft_bin_for_frequency(float freq_hz) {
    const float normalized = freq_hz / static_cast<float>(WIDEBAND_SAMPLE_RATE);
    long long bin =
        std::llround(normalized * static_cast<float>(WidebandMonitor::FFT_SIZE));
    bin %= static_cast<long long>(WidebandMonitor::FFT_SIZE);
    if (bin < 0) bin += static_cast<long long>(WidebandMonitor::FFT_SIZE);
    return static_cast<size_t>(bin);
}

float power_db(const std::complex<float>& value) {
    return 10.0f * std::log10(std::norm(value) + FFT_EPSILON);
}

float band_power_db(const std::vector<std::complex<float>>& spectrum,
                    float center_freq_hz) {
    const float start_hz = center_freq_hz - CHANNEL_HALF_BW_HZ;
    const float stop_hz  = center_freq_hz + CHANNEL_HALF_BW_HZ;

    const size_t start_bin = fft_bin_for_frequency(start_hz);
    const size_t stop_bin  = fft_bin_for_frequency(stop_hz);

    double power = 0.0;
    size_t bins = 0;

    auto accumulate_bin = [&](size_t index) {
        power += std::norm(spectrum[index]);
        ++bins;
    };

    if (start_bin <= stop_bin) {
        for (size_t i = start_bin; i <= stop_bin; ++i) accumulate_bin(i);
    } else {
        for (size_t i = start_bin; i < spectrum.size(); ++i) accumulate_bin(i);
        for (size_t i = 0; i <= stop_bin; ++i) accumulate_bin(i);
    }

    const double avg_power = (bins > 0) ? (power / static_cast<double>(bins)) : FFT_EPSILON;
    return 10.0f * std::log10(static_cast<float>(avg_power) + FFT_EPSILON);
}

} // namespace

WidebandMonitor::WidebandMonitor()
    : ring_buffer_(FFT_SIZE)
    , smoothed_power_db_{}
    , peak_delta_db_{}
    , phase_diff_{}
    , mixer_{}
    , mixer_step_{}
    , decim_accum_{}
    , decim_phase_{}
    , receivers_{}
    , decoders_{}
    , voice_detected_{}
    , active_parts_{}
    , packets_seen_{}
    , fft_plot_db_{}
    , waterfall_history_(WATERFALL_HISTORY * FFT_PLOT_BINS, -120.0f)
    , channel_views_{}
    , write_pos_(0)
    , buffered_samples_(0)
    , sample_counter_(0)
    , have_history_(false)
    , visual_ready_(false)
    , waterfall_head_(0)
    , waterfall_rows_filled_(0)
    , noise_floor_db_(0.0f) {
    fft_plot_db_.fill(-120.0f);

    for (size_t i = 0; i < US_DECT_CHANNELS.size(); ++i) {
        const float offset_hz =
            static_cast<float>(static_cast<int64_t>(US_DECT_CHANNELS[i].freq_hz) -
                               static_cast<int64_t>(WIDEBAND_CENTER_FREQ_HZ));
        const float angle = -2.0f * PI_F * offset_hz / static_cast<float>(WIDEBAND_SAMPLE_RATE);

        mixer_[i] = {1.0f, 0.0f};
        mixer_step_[i] = {std::cos(angle), std::sin(angle)};
        decim_accum_[i] = {};
        decim_phase_[i] = 0;

        channel_views_[i].channel_number = US_DECT_CHANNELS[i].number;
        channel_views_[i].freq_hz = US_DECT_CHANNELS[i].freq_hz;

        decoders_[i] = std::make_unique<PacketDecoder>(
            [this, i](const PartInfo parts[], int count) {
                on_channel_update(i, parts, count);
            },
            [this, i](int rx_id, const int16_t* pcm, size_t count) {
                on_voice_packet(i, rx_id, pcm, count);
            });

        receivers_[i] = std::make_unique<PacketReceiver>(
            [this, i](const ReceivedPacket& pkt) {
                {
                    std::lock_guard<std::mutex> lock(status_mutex_);
                    ++packets_seen_[i];
                }
                decoders_[i]->process_packet(pkt);
            },
            [this, i](int rx_id) {
                decoders_[i]->notify_lost(rx_id);
            });
    }
}

void WidebandMonitor::set_dc_block(bool enabled) {
    dc_block_enabled_ = enabled;
    if (!enabled) dc_blocker_.reset();
}

bool WidebandMonitor::dc_block_enabled() const {
    return dc_block_enabled_;
}

void WidebandMonitor::ingest(const std::complex<float>* samples, size_t n) {
    if (n == 0) return;

    // Apply DC blocker if enabled (filter into reusable scratch buffer)
    if (dc_block_enabled_) {
        if (dc_buf_.size() < n) dc_buf_.resize(n);
        for (size_t i = 0; i < n; ++i)
            dc_buf_[i] = dc_blocker_.process(samples[i]);
        samples = dc_buf_.data();
    }

    {
        std::lock_guard<std::mutex> lock(ring_mutex_);

        if (n >= FFT_SIZE) {
            const auto* start = samples + (n - FFT_SIZE);
            std::copy(start, start + FFT_SIZE, ring_buffer_.begin());
            write_pos_ = 0;
            buffered_samples_ = FFT_SIZE;
        } else {
            for (size_t i = 0; i < n; ++i) {
                ring_buffer_[write_pos_] = samples[i];
                write_pos_ = (write_pos_ + 1) % FFT_SIZE;
            }

            buffered_samples_ = std::min(buffered_samples_ + n, FFT_SIZE);
        }
    }

    for (size_t i = 0; i < n; ++i) {
        process_sample(samples[i]);
    }
}

void WidebandMonitor::process_sample(std::complex<float> sample) noexcept {
    for (size_t i = 0; i < US_DECT_CHANNELS.size(); ++i) {
        const std::complex<float> mixed = sample * mixer_[i];
        mixer_[i] *= mixer_step_[i];
        decim_accum_[i] += mixed;

        if (++decim_phase_[i] == 4) {
            decim_phase_[i] = 0;
            const std::complex<float> decimated = decim_accum_[i] * 0.25f;
            decim_accum_[i] = {};

            const float phase = phase_diff_[i].process(decimated);
            receivers_[i]->process_sample(phase);
        }
    }

    ++sample_counter_;
    if ((sample_counter_ & 0x0FFFu) == 0) {
        for (auto& osc : mixer_) {
            const float mag = std::abs(osc);
            if (mag > 0.0f) osc /= mag;
        }
    }
}

bool WidebandMonitor::snapshot_latest_block(std::vector<std::complex<float>>& out,
                                            size_t& buffered_samples) {
    std::lock_guard<std::mutex> lock(ring_mutex_);
    buffered_samples = buffered_samples_;
    if (buffered_samples_ < FFT_SIZE) {
        return false;
    }

    out.resize(FFT_SIZE);
    for (size_t i = 0; i < FFT_SIZE; ++i) {
        out[i] = ring_buffer_[(write_pos_ + i) % FFT_SIZE];
    }
    return true;
}

void WidebandMonitor::snapshot_channel_state(
    std::array<ChannelDecoderSnapshot, US_DECT_CHANNELS.size()>& out) const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    for (size_t i = 0; i < US_DECT_CHANNELS.size(); ++i) {
        out[i].voice_detected   = voice_detected_[i];
        out[i].qt_synced        = channel_views_[i].qt_synced;
        out[i].active_parts     = active_parts_[i];
        out[i].packets_seen     = packets_seen_[i];
        out[i].voice_frames_ok  = channel_views_[i].voice_frames_ok;
        out[i].voice_xcrc_fail  = channel_views_[i].voice_xcrc_fail;
        out[i].voice_skipped    = channel_views_[i].voice_skipped;
    }
}

void WidebandMonitor::on_channel_update(size_t channel_index,
                                        const PartInfo parts[],
                                        int count) noexcept {
    bool voice = false;
    bool qt    = false;
    uint64_t vf_ok = 0, vf_fail = 0, vf_skip = 0;
    for (int i = 0; i < count; ++i) {
        if (parts[i].voice_present) voice = true;
        if (parts[i].qt_synced)     qt    = true;
        vf_ok   += parts[i].voice_frames_ok;
        vf_fail += parts[i].voice_xcrc_fail;
        vf_skip += parts[i].voice_skipped;
    }

    std::lock_guard<std::mutex> lock(status_mutex_);
    active_parts_[channel_index] = count;
    voice_detected_[channel_index] = voice;
    channel_views_[channel_index].voice_frames_ok  = vf_ok;
    channel_views_[channel_index].voice_xcrc_fail   = vf_fail;
    channel_views_[channel_index].voice_skipped     = vf_skip;
    channel_views_[channel_index].qt_synced          = qt;
}

void WidebandMonitor::on_voice_packet(size_t channel_index, int /*rx_id*/,
                                      const int16_t* pcm, size_t count) noexcept {
    std::lock_guard<std::mutex> lock(audio_mutex_);
    if (!audio_out_) return;

    int selected = audio_channel_;
    if (selected < 0) {
        // Auto mode: lock onto the first channel that delivers voice
        audio_channel_ = static_cast<int>(channel_index);
        selected = audio_channel_;
    }
    if (static_cast<int>(channel_index) == selected)
        audio_out_->write_samples(pcm, count);
}

void WidebandMonitor::set_audio_output(AudioOutput* audio) {
    std::lock_guard<std::mutex> lock(audio_mutex_);
    audio_out_ = audio;
}

void WidebandMonitor::set_audio_channel(int channel_index) {
    std::lock_guard<std::mutex> lock(audio_mutex_);
    audio_channel_ = channel_index;
}

int WidebandMonitor::audio_channel() const {
    std::lock_guard<std::mutex> lock(audio_mutex_);
    return audio_channel_;
}

bool WidebandMonitor::update_visuals() {
    std::vector<std::complex<float>> spectrum;
    size_t buffered_samples = 0;
    if (!snapshot_latest_block(spectrum, buffered_samples)) {
        std::lock_guard<std::mutex> lock(visual_mutex_);
        visual_ready_ = false;
        return false;
    }

    for (size_t i = 0; i < spectrum.size(); ++i) {
        const float window = 0.5f - 0.5f * std::cos((2.0f * PI_F * static_cast<float>(i)) /
                                                    static_cast<float>(spectrum.size() - 1));
        spectrum[i] *= window;
    }

    fft_inplace(spectrum);

    std::array<ChannelDecoderSnapshot, US_DECT_CHANNELS.size()> channel_state{};
    snapshot_channel_state(channel_state);

    std::array<float, US_DECT_CHANNELS.size()> frame_power_db{};
    for (size_t i = 0; i < US_DECT_CHANNELS.size(); ++i) {
        const float offset_hz =
            static_cast<float>(static_cast<int64_t>(US_DECT_CHANNELS[i].freq_hz) -
                               static_cast<int64_t>(WIDEBAND_CENTER_FREQ_HZ));
        frame_power_db[i] = band_power_db(spectrum, offset_hz);
        if (!have_history_) {
            smoothed_power_db_[i] = frame_power_db[i];
        } else {
            smoothed_power_db_[i] = 0.85f * smoothed_power_db_[i] + 0.15f * frame_power_db[i];
        }
    }
    have_history_ = true;

    auto median_power = smoothed_power_db_;
    const auto median_it = median_power.begin() + (median_power.size() / 2);
    std::nth_element(median_power.begin(), median_it, median_power.end());
    const float noise_floor_db = *median_it;

    // Compute raw FFT bins for this frame
    std::array<float, FFT_PLOT_BINS> fft_plot_raw{};
    for (size_t col = 0; col < FFT_PLOT_BINS; ++col) {
        const size_t start = (col * FFT_SIZE) / FFT_PLOT_BINS;
        const size_t stop  = ((col + 1) * FFT_SIZE) / FFT_PLOT_BINS;

        float max_db = -120.0f;
        for (size_t bin = start; bin < stop; ++bin) {
            const size_t shifted_index = (bin + FFT_SIZE / 2) % FFT_SIZE;
            max_db = std::max(max_db, power_db(spectrum[shifted_index]));
        }
        fft_plot_raw[col] = max_db;
    }

    {
        std::lock_guard<std::mutex> lock(visual_mutex_);
        visual_ready_ = true;
        noise_floor_db_ = noise_floor_db;

        // IIR smooth FFT display: fast attack (0.4), slow decay (0.15)
        // This stabilises the trace without hiding transient signals.
        for (size_t col = 0; col < FFT_PLOT_BINS; ++col) {
            if (fft_plot_raw[col] > fft_plot_db_[col])
                fft_plot_db_[col] = 0.6f * fft_plot_db_[col] + 0.4f * fft_plot_raw[col];
            else
                fft_plot_db_[col] = 0.85f * fft_plot_db_[col] + 0.15f * fft_plot_raw[col];
        }

        // Waterfall uses raw (un-smoothed) data so it stays crisp
        const size_t row_offset = waterfall_head_ * FFT_PLOT_BINS;
        std::copy(fft_plot_raw.begin(), fft_plot_raw.end(),
                  waterfall_history_.begin() + static_cast<std::ptrdiff_t>(row_offset));
        waterfall_head_ = (waterfall_head_ + 1) % WATERFALL_HISTORY;
        waterfall_rows_filled_ = std::min(waterfall_rows_filled_ + 1, WATERFALL_HISTORY);

        for (size_t i = 0; i < US_DECT_CHANNELS.size(); ++i) {
            const float delta_db = std::max(0.0f, smoothed_power_db_[i] - noise_floor_db_);
            peak_delta_db_[i] = std::max(peak_delta_db_[i] * 0.96f, delta_db);

            channel_views_[i].smoothed_power_db = smoothed_power_db_[i];
            channel_views_[i].relative_power_db = delta_db;
            channel_views_[i].voice_detected = channel_state[i].voice_detected;
            channel_views_[i].qt_synced = channel_state[i].qt_synced;
            channel_views_[i].active_parts = channel_state[i].active_parts;
            channel_views_[i].packets_seen = channel_state[i].packets_seen;
            channel_views_[i].voice_frames_ok = channel_state[i].voice_frames_ok;
            channel_views_[i].voice_xcrc_fail = channel_state[i].voice_xcrc_fail;
            channel_views_[i].voice_skipped = channel_state[i].voice_skipped;

            // Hysteresis: activate at ≥3 dB above noise, deactivate below 1.5 dB
            bool has_signal = channel_state[i].voice_detected || channel_state[i].active_parts > 0;
            if (channel_views_[i].active)
                channel_views_[i].active = (delta_db >= 1.5f) || has_signal;
            else
                channel_views_[i].active = (delta_db >= 3.0f) || has_signal;
        }
    }

    return true;
}

WidebandSnapshot WidebandMonitor::snapshot() const {
    WidebandSnapshot out;

    {
        std::lock_guard<std::mutex> ring_lock(ring_mutex_);
        out.buffered_samples = buffered_samples_;
    }

    std::lock_guard<std::mutex> lock(visual_mutex_);
    out.ready = visual_ready_;
    out.noise_floor_db = noise_floor_db_;
    out.channels = channel_views_;
    out.waterfall_rows = WATERFALL_HISTORY;
    out.waterfall_cols = FFT_PLOT_BINS;
    out.fft_db.assign(fft_plot_db_.begin(), fft_plot_db_.end());
    out.waterfall_db.assign(WATERFALL_HISTORY * FFT_PLOT_BINS, -120.0f);

    if (waterfall_rows_filled_ == 0) {
        return out;
    }

    const bool full = waterfall_rows_filled_ == WATERFALL_HISTORY;
    const size_t source_start = full ? waterfall_head_ : 0;
    const size_t padding_rows = WATERFALL_HISTORY - waterfall_rows_filled_;

    for (size_t row = 0; row < waterfall_rows_filled_; ++row) {
        const size_t src_row = full ? ((source_start + row) % WATERFALL_HISTORY) : row;
        const size_t dst_row = padding_rows + row;
        std::copy_n(
            waterfall_history_.begin() + static_cast<std::ptrdiff_t>(src_row * FFT_PLOT_BINS),
            FFT_PLOT_BINS,
            out.waterfall_db.begin() + static_cast<std::ptrdiff_t>(dst_row * FFT_PLOT_BINS));
    }

    return out;
}

void WidebandMonitor::render_waiting_screen(size_t buffered_samples) const {
    std::printf("\x1b[H\x1b[J");
    std::printf("DeDECTive wideband monitor\n");
    std::printf("  Center: %.3f MHz  Sample rate: %.3f Msps\n",
                WIDEBAND_CENTER_FREQ_HZ / 1e6,
                WIDEBAND_SAMPLE_RATE / 1e6);
    std::printf("  Filling analysis buffer: %zu / %zu samples\n\n",
                buffered_samples, FFT_SIZE);
    std::printf("Waiting for enough IQ samples to render the full-band activity view...\n");
    std::fflush(stdout);
}

void WidebandMonitor::render_frame() {
    if (!update_visuals()) {
        WidebandSnapshot current = snapshot();
        render_waiting_screen(current.buffered_samples);
        return;
    }

    const WidebandSnapshot current = snapshot();

    std::printf("\x1b[H\x1b[J");
    std::printf("DeDECTive wideband monitor\n");
    std::printf("  Center: %.3f MHz  Sample rate: %.3f Msps  FFT: %zu samples\n",
                WIDEBAND_CENTER_FREQ_HZ / 1e6,
                WIDEBAND_SAMPLE_RATE / 1e6,
                FFT_SIZE);
    std::printf("  Bars show channel power relative to the current band median.\n");
    std::printf("  Voice is protocol-derived from DECT packet decode, not audio decode.\n\n");

    for (const auto& channel : current.channels) {
        const int bar_len = std::clamp(
            static_cast<int>(std::lround(channel.relative_power_db * 2.0f)), 0, 28);

        char bar[29];
        std::fill(std::begin(bar), std::end(bar) - 1, '.');
        bar[28] = '\0';
        for (int j = 0; j < bar_len; ++j) bar[j] = '#';

        std::printf("  Ch %-2d %.3f MHz  %+5.1f dB  [%s]  parts:%d  voice:%s  pkts:%llu\n",
                    channel.channel_number,
                    channel.freq_hz / 1e6,
                    channel.relative_power_db,
                    bar,
                    channel.active_parts,
                    channel.voice_detected ? "YES" : " no",
                    static_cast<unsigned long long>(channel.packets_seen));
    }

    std::printf("\nPress Ctrl-C to stop.\n");
    std::fflush(stdout);
}

void WidebandMonitor::fft_inplace(std::vector<std::complex<float>>& data) {
    const size_t n = data.size();

    for (size_t i = 1, j = 0; i < n; ++i) {
        size_t bit = n >> 1;
        for (; j & bit; bit >>= 1) {
            j ^= bit;
        }
        j ^= bit;
        if (i < j) std::swap(data[i], data[j]);
    }

    for (size_t len = 2; len <= n; len <<= 1) {
        const float angle = -2.0f * PI_F / static_cast<float>(len);
        const std::complex<float> wlen(std::cos(angle), std::sin(angle));

        for (size_t i = 0; i < n; i += len) {
            std::complex<float> w(1.0f, 0.0f);
            for (size_t j = 0; j < len / 2; ++j) {
                const std::complex<float> u = data[i + j];
                const std::complex<float> v = data[i + j + len / 2] * w;
                data[i + j] = u + v;
                data[i + j + len / 2] = u - v;
                w *= wlen;
            }
        }
    }
}

} // namespace dedective
