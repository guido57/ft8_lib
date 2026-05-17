"""
PlatformIO extra_script: utils/pio_sources.py
Adds FT8 core sources that live outside src/.

Note: src/decode_ft8.c is now built by PlatformIO's default src builder,
so it is intentionally not included here to avoid duplicate compilation.
"""
Import("env")

env.BuildSources(
    "$BUILD_DIR/ft8_core",
    "$PROJECT_DIR",
    src_filter=" ".join([
        "-<*>",              # exclude everything from project root by default
        "+<ft8_decode/*.c>",        # LDPC, pack/unpack, CRC, text, encode, decode, decoder_api
        "+<fft/*.c>",        # kiss_fft, kiss_fftr
        "+<common/*.c>",     # wave.c (no-op on ESP32 via #ifndef guard)
    ])
)
