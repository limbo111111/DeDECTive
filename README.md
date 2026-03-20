# DeDECTive

`DeDECTive` is a Linux-based DECT 6.0 scanner for the HackRF One.

It currently supports:
- scanning the full US DECT channel set
- real-time DECT activity detection
- RFP detection
- PP detection
- voice presence detection
- wideband realtime activity visualization across the full US DECT band
- wideband per-channel voice indication from DECT packet decode

Current limitation:
- voice decode does not work yet

## Reference implementation credit

`DeDECTive` was guided by `gr-dect2` as the reference implementation for the DECT protocol pipeline used here. In particular, the phase-difference demodulation, packet reception flow, packet decode behavior, and related protocol constants were ported and adapted from that project to make this HackRF-based Linux scanner work.

## Build

```bash
cmake -S . -B build
cmake --build build -j
```

## Run

```bash
./build/dedective -h
```

Sequential scanner:

```bash
./build/dedective
```

Wideband activity and voice monitor:

```bash
./build/dedective -W -l
```

This mode captures the whole US DECT band in one HackRF pass, shows live per-channel activity bars, and reports whether voice is currently detected on each channel from DECT packet decode state. It is still not a voice decoder.
