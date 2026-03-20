# DeDECTive

`DeDECTive` is a Linux-based DECT 6.0 scanner and voice decoder for the HackRF One. It provides a Dear ImGui GUI with wideband spectrum visualization and narrowband voice decode in a single application — no GNU Radio required.

![Wideband Scanner](main.png)

## Features

### Wideband Scanner
- Captures the entire US DECT band (10 channels) in a single HackRF pass at 18.432 Msps
- Live FFT and waterfall display with adjustable dB range sliders
- Per-channel activity detection with hysteresis for stable readings
- Per-channel DECT packet decode showing RFP/PP parts, voice presence, and Qt sync status
- Channel details table with one-click **Tune** buttons to switch to narrowband

### Narrowband Voice Decode
- Tunes to a single DECT channel for full voice decode
- G.721 ADPCM decoding of both RFP (base station) and PP (handset) audio
- RFP + PP audio mixed into a single stream so both sides of the call are heard
- Real-time PulseAudio playback with volume control
- Timeslot tracking (slots 0–11 downlink, 12–23 uplink)
- Sequence gap filling with G.721 comfort noise for smooth audio across missed frames

![Narrowband Voice Decode](call_following.png)

### Call Following
- Automatic channel handoff tracking — when a call moves channels, DeDECTive follows it
- If voice stops in narrowband, auto-switches back to wideband scan after 2 seconds
- When voice is detected on any channel in wideband, auto-tunes to narrowband

### Signal Processing
- DC spike correction (IIR high-pass filter) to remove HackRF's DC offset
- FFT temporal smoothing with fast attack / slow decay
- Configurable LNA, VGA gain, and HackRF amplifier

## Build

Dependencies: `libhackrf`, `libpulse-simple`, `SDL2`, `imgui`, `OpenGL`

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

## Run

### GUI (recommended)

```bash
./build/dedective_gui
```

### CLI

Sequential channel scanner:

```bash
./build/dedective
```

Wideband activity monitor (terminal):

```bash
./build/dedective -W -l
```

## Architecture

The codebase is split into a reusable core library and two frontends:

- **dedective_core** — static library containing the HackRF interface, DECT packet receiver/decoder, wideband monitor, audio output, and G.721 codec
- **dedective** — CLI frontend
- **dedective_gui** — Dear ImGui + SDL2 + OpenGL frontend

## Reference

The DECT protocol pipeline (phase-difference demodulation, packet reception, A/B-field decode, scramble tables) was ported and adapted from [gr-dect2](https://github.com/pavelyazev/gr-dect2) by Pavel Yazev.
