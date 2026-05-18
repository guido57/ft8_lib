#ifndef FT8_CONFIG_H
#define FT8_CONFIG_H

// Compile-time tuning knobs for the FT8 decode core.
// Override any of these via -D on the compiler command line or in your
// platform build system (e.g. ESP-IDF CMakeLists target_compile_definitions).
//
// Desktop defaults (full sensitivity, no memory pressure):
//   FT8_MIN_SCORE          10
//   FT8_MAX_CANDIDATES    120
//   FT8_LDPC_ITERATIONS    20
//   FT8_MAX_DECODED_MSGS 1000
//   FT8_FREQ_OSR            2
//   FT8_TIME_OSR            2
//   FT8_SNR_RAW_SCALE     0.80
//   FT8_SNR_SCORE_SCALE   0.00
//   FT8_SNR_OFFSET      -29.00
//   FT8_SNR_MIN_DB      -30.00
//   FT8_SNR_MAX_DB       20.00
//
// Suggested ESP32-S3 starting profile (tune up from here):
//   -DFT8_MAX_CANDIDATES=50
//   -DFT8_LDPC_ITERATIONS=10
//   -DFT8_MAX_DECODED_MSGS=200

#ifndef FT8_MIN_SCORE
#define FT8_MIN_SCORE 10
#endif

#ifndef FT8_MAX_CANDIDATES
#define FT8_MAX_CANDIDATES 120
#endif

#ifndef FT8_LDPC_ITERATIONS
#define FT8_LDPC_ITERATIONS 20
#endif

#ifndef FT8_MAX_DECODED_MSGS
#define FT8_MAX_DECODED_MSGS 1000
#endif

#ifndef FT8_FREQ_OSR
#define FT8_FREQ_OSR 2
#endif

#ifndef FT8_TIME_OSR
#define FT8_TIME_OSR 2
#endif

#ifndef FT8_SNR_RAW_SCALE
#define FT8_SNR_RAW_SCALE 0.80f
#endif

#ifndef FT8_SNR_SCORE_SCALE
#define FT8_SNR_SCORE_SCALE 0.00f
#endif

#ifndef FT8_SNR_OFFSET
#define FT8_SNR_OFFSET -29.00f
#endif

#ifndef FT8_SNR_MIN_DB
#define FT8_SNR_MIN_DB -30.00f
#endif

#ifndef FT8_SNR_MAX_DB
#define FT8_SNR_MAX_DB 20.00f
#endif

#endif // FT8_CONFIG_H
