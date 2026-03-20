# DeDECTive

`DeDECTive` is a Linux-based DECT 6.0 scanner for the HackRF One.

It currently supports:
- scanning the full US DECT channel set
- real-time DECT activity detection
- RFP detection
- PP detection
- voice presence detection

Current limitation:
- voice decode does not work yet

## Build

```bash
cmake -S . -B build
cmake --build build -j
```

## Run

```bash
./build/dedective -h
```
