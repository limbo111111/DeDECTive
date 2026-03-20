/*
 * DeDECTive — US DECT 6.0 Scanner
 *
 * Scans all 10 US DECT 6.0 channels (1921.536–1937.088 MHz) using a HackRF One,
 * detects DECT sync words, decodes the A-field to identify RFP and PP devices.
 *
 * Build:  see CMakeLists.txt
 * Usage:  dedective [options]
 *   -g <lna>   LNA gain 0-40 dB (default: 32)
 *   -v <vga>   VGA gain 0-62 dB (default: 20)
 *   -a         Enable HackRF RF amplifier (+14 dB, use carefully)
 *   -d <secs>  Dwell time per channel in seconds (default: 3)
 *   -c <ch>    Scan only this channel number 0-5 (default: all)
 *   -l         Loop scan continuously until Ctrl-C
 *   -W         Wideband monitor of the whole US DECT band
 */

#include "hackrf_source.h"
#include "phase_diff.h"
#include "packet_receiver.h"
#include "packet_decoder.h"
#include "dect_channels.h"
#include "wideband_monitor.h"
#include "audio_output.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>
#include <string>
#include <vector>
#include <iomanip>
#include <sstream>
#include <iostream>

using namespace dedective;

// ── Global signal handler for clean Ctrl-C exit ───────────────────────────────
static std::atomic<bool> g_running{true};
static void signal_handler(int) { g_running = false; }

// ── Per-channel scanning context ──────────────────────────────────────────────
struct ScanContext {
    PhaseDiff      phase_diff;
    PacketReceiver receiver;
    PacketDecoder  decoder;
    AudioOutput*   audio_out = nullptr;
    std::mutex     print_mutex;
    uint64_t       cur_freq_hz  = 0;
    int            cur_channel  = -1;
    uint64_t       packets_seen = 0;

    ScanContext(int channel, uint64_t freq_hz, AudioOutput* audio = nullptr)
        : receiver(
            // on_packet
            [this](const ReceivedPacket& pkt) {
                ++packets_seen;
                decoder.process_packet(pkt);
            },
            // on_lost
            [this](int rx_id) {
                decoder.notify_lost(rx_id);
            }
          )
        , decoder(
            // on_update
            [this](const PartInfo parts[], int count) {
                print_parts(parts, count);
            },
            // on_voice
            [this](int rx_id, const int16_t* pcm, size_t count) {
                if (audio_out)
                    audio_out->write_samples(pcm, count);
            }
          )
        , audio_out(audio)
        , cur_freq_hz(freq_hz)
        , cur_channel(channel)
    {}

    void print_parts(const PartInfo* parts, int count) {
        std::lock_guard<std::mutex> lock(print_mutex);
        std::printf("\r  [Ch %d  %.3f MHz]  %d device(s):\n",
                    cur_channel, cur_freq_hz / 1e6, count);
        for (int i = 0; i < count; ++i) {
            const PartInfo& p = parts[i];
            int crc_pct = (p.packets_ok + p.packets_bad_crc > 0)
                ? (int)(100 * p.packets_ok / (p.packets_ok + p.packets_bad_crc))
                : 0;
            std::printf("    rx%d  ID: %-10s  Type: %-3s  Voice: %-3s"
                        "  Pkts: %4llu  CRC ok: %3d%%\n",
                        p.rx_id,
                        p.part_id_valid ? p.part_id_hex().c_str() : "??????????",
                        p.type_str().c_str(),
                        p.voice_present ? "YES" : "no",
                        (unsigned long long)p.packets_ok,
                        crc_pct);
        }
        std::fflush(stdout);
    }
};

// ── IQ callback from HackRF ───────────────────────────────────────────────────
static void iq_callback(ScanContext* ctx,
                        const std::complex<float>* samples,
                        size_t n)
{
    for (size_t i = 0; i < n; ++i) {
        float phase = ctx->phase_diff.process(samples[i]);
        ctx->receiver.process_sample(phase);
    }
}

// ── Usage ─────────────────────────────────────────────────────────────────────
static void print_usage(const char* prog) {
    std::printf(
        "Usage: %s [options]\n"
        "\nOptions:\n"
        "  -g <lna>   LNA gain  0-40 dB, steps of 8  (default: 32)\n"
        "  -v <vga>   VGA gain  0-62 dB, steps of 2  (default: 20)\n"
        "  -a         Enable HackRF RF amplifier (+14 dB)\n"
        "  -d <secs>  Dwell time per channel, seconds (default: 3)\n"
        "  -c <ch>    Scan only channel 0-9           (default: all)\n"
        "  -l         Loop continuously until Ctrl-C\n"
        "  -W         Wideband mode: monitor all channels at once\n"
        "  -V         Enable voice decode (audio output via PulseAudio)\n"
        "  -h         Show this help\n"
        "\nUS DECT 6.0 Channels:\n",
        prog);
    for (auto& ch : US_DECT_CHANNELS)
        std::printf("  %d  %s\n", ch.number, ch.label);
    std::printf("\nSample rate: %u Hz  (4 samples/symbol)\n", SAMPLE_RATE);
    std::printf("Wideband mode sample rate: %u Hz\n", WIDEBAND_SAMPLE_RATE);
}

// ── main ──────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[]) {
    // Defaults
    uint32_t lna_gain    = 32;
    uint32_t vga_gain    = 20;
    bool     amp_enable  = false;
    int      dwell_secs  = 3;
    bool     dwell_set   = false;
    int      fixed_chan  = -1;  // -1 = scan all
    bool     loop        = false;
    bool     wideband    = false;
    bool     voice_en    = false;

    // Parse arguments
    for (int i = 1; i < argc; ++i) {
        if      (strcmp(argv[i], "-g") == 0 && i+1 < argc) lna_gain   = (uint32_t)atoi(argv[++i]);
        else if (strcmp(argv[i], "-v") == 0 && i+1 < argc) vga_gain   = (uint32_t)atoi(argv[++i]);
        else if (strcmp(argv[i], "-d") == 0 && i+1 < argc) { dwell_secs = atoi(argv[++i]); dwell_set = true; }
        else if (strcmp(argv[i], "-c") == 0 && i+1 < argc) fixed_chan  = atoi(argv[++i]);
        else if (strcmp(argv[i], "-a") == 0)               amp_enable  = true;
        else if (strcmp(argv[i], "-l") == 0)               loop        = true;
        else if (strcmp(argv[i], "-W") == 0)               wideband    = true;
        else if (strcmp(argv[i], "-V") == 0)               voice_en    = true;
        else if (strcmp(argv[i], "-h") == 0) { print_usage(argv[0]); return 0; }
        else { std::fprintf(stderr, "Unknown option: %s\n", argv[i]); print_usage(argv[0]); return 1; }
    }

    if (dwell_secs <= 0) {
        std::fprintf(stderr, "Duration must be positive.\n");
        return 1;
    }
    if (wideband && fixed_chan >= 0) {
        std::fprintf(stderr, "Wideband mode captures all channels at once; do not combine -W with -c.\n");
        return 1;
    }
    if (wideband && !loop && !dwell_set) {
        dwell_secs = 10;
    }

    signal(SIGINT,  signal_handler);
    signal(SIGTERM, signal_handler);

    std::printf("DeDECTive v0.1 - US DECT 6.0 Scanner\n");
    std::printf("  LNA gain: %u dB  VGA gain: %u dB  Amp: %s\n",
                lna_gain, vga_gain, amp_enable ? "ON" : "off");
    if (wideband)
        std::printf("  Duration: %d s\n", dwell_secs);
    else
        std::printf("  Dwell:    %d s per channel\n", dwell_secs);
    std::printf("  Mode:     %s%s\n\n",
                loop ? "continuous loop" : "single pass",
                wideband ? " wideband monitor" : "");

    // Build channel list for this scan pass
    std::vector<DectChannel> channels;
    if (fixed_chan >= 0 && fixed_chan < (int)US_DECT_CHANNELS.size()) {
        channels.push_back(US_DECT_CHANNELS[fixed_chan]);
    } else {
        for (auto& ch : US_DECT_CHANNELS) channels.push_back(ch);
    }

    // Open HackRF
    HackrfSource hackrf;
    std::printf("Opening HackRF... ");
    std::fflush(stdout);
    if (!hackrf.open()) {
        std::fprintf(stderr, "FAILED: %s\n", hackrf.last_error());
        return 1;
    }
    std::printf("OK\n");

    const uint32_t sample_rate = wideband ? WIDEBAND_SAMPLE_RATE : SAMPLE_RATE;
    if (!hackrf.set_sample_rate(sample_rate)) {
        std::fprintf(stderr, "set_sample_rate failed: %s\n", hackrf.last_error());
        return 1;
    }
    if (!hackrf.set_lna_gain(lna_gain) || !hackrf.set_vga_gain(vga_gain)) {
        std::fprintf(stderr, "set_gain failed: %s\n", hackrf.last_error());
        return 1;
    }
    if (!hackrf.set_amp_enable(amp_enable)) {
        std::fprintf(stderr, "set_amp failed: %s\n", hackrf.last_error());
        return 1;
    }

    if (wideband) {
        WidebandMonitor monitor;
        AudioOutput audio;
        if (voice_en) {
            if (audio.start()) {
                monitor.set_audio_output(&audio);
                std::printf("Voice decode enabled (PulseAudio output)\n");
            } else {
                std::fprintf(stderr, "Warning: could not open PulseAudio\n");
            }
        }
        std::printf("Wideband center frequency: %.3f MHz\n", WIDEBAND_CENTER_FREQ_HZ / 1e6);
        if (!hackrf.set_freq(WIDEBAND_CENTER_FREQ_HZ)) {
            std::fprintf(stderr, "set_freq failed: %s\n", hackrf.last_error());
            return 1;
        }
        if (!hackrf.start([&monitor](const std::complex<float>* samples, size_t n) {
                monitor.ingest(samples, n);
            })) {
            std::fprintf(stderr, "start failed: %s\n", hackrf.last_error());
            return 1;
        }

        auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(dwell_secs);
        while (g_running && (loop || std::chrono::steady_clock::now() < deadline)) {
            monitor.render_frame();
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }

        hackrf.stop();
        std::printf("\nDone.\n");
        return 0;
    }

    // Scan loop
    AudioOutput scan_audio;
    if (voice_en && !wideband) {
        if (scan_audio.start())
            std::printf("Voice decode enabled (PulseAudio output)\n");
        else
            std::fprintf(stderr, "Warning: could not open PulseAudio\n");
    }

    do {
        for (auto& chan : channels) {
            if (!g_running) break;

            std::printf("Scanning channel %d  %s  (%d s)...\n",
                        chan.number, chan.label, dwell_secs);
            std::fflush(stdout);

            // Fresh context for each channel (reset state)
            AudioOutput* audio_ptr = (voice_en && scan_audio.is_running())
                                   ? &scan_audio : nullptr;
            ScanContext ctx(chan.number, chan.freq_hz, audio_ptr);

            if (!hackrf.set_freq(chan.freq_hz)) {
                std::fprintf(stderr, "  set_freq failed: %s — skipping\n",
                             hackrf.last_error());
                continue;
            }

            if (!hackrf.start([&ctx](const std::complex<float>* s, size_t n) {
                    iq_callback(&ctx, s, n);
                })) {
                std::fprintf(stderr, "  start failed: %s — skipping\n",
                             hackrf.last_error());
                continue;
            }

            // Dwell — if we detect activity mid-scan, extend the window
            auto deadline = std::chrono::steady_clock::now()
                          + std::chrono::seconds(dwell_secs);
            bool activity_reported = false;
            while (g_running && std::chrono::steady_clock::now() < deadline) {
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                if (!activity_reported && ctx.packets_seen > 0) {
                    activity_reported = true;
                    std::printf("  *** DECT activity! Extending dwell...\n");
                    std::fflush(stdout);
                    // Extend by another dwell_secs when we first hit activity
                    deadline = std::chrono::steady_clock::now()
                             + std::chrono::seconds(dwell_secs);
                }
            }

            hackrf.stop();

            if (ctx.packets_seen > 0)
                std::printf("  Summary ch%d: %llu sync locks, %d active device(s)\n",
                            chan.number,
                            (unsigned long long)ctx.packets_seen,
                            ctx.decoder.active_count());
            else
                std::printf("  No DECT sync detected on this channel.\n");
        }
    } while (loop && g_running);

    std::printf("\nDone.\n");
    return 0;
}
