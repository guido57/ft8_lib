#ifndef FT8_DECODER_API_H
#define FT8_DECODER_API_H

#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

// Metadata passed to one FT8/FT4 decode call.
typedef struct {
    bool is_ft8;
    float base_freq_mhz;
    struct tm utc;
    double utc_frac_sec;
} ft8_decode_context_t;

// Decode a single audio slot from floating-point samples.
// sample_rate is in Hz, base_freq_mhz is in MHz.
int ft8_decode_slot(const float* signal, int sample_rate, int num_samples, const ft8_decode_context_t* ctx);

// Opaque handle for streaming slot ingest.
typedef struct ft8_stream_decoder ft8_stream_decoder_t;

// Open a streaming decoder for one slot.
ft8_stream_decoder_t* ft8_stream_open(int sample_rate, const ft8_decode_context_t* ctx);

// Append samples while receiving audio.
int ft8_stream_append_float(ft8_stream_decoder_t* stream, const float* signal, int num_samples);
int ft8_stream_append_i16(ft8_stream_decoder_t* stream, const int16_t* signal, int num_samples);

// Finalize one slot: run candidate search + decode.
int ft8_stream_finalize(ft8_stream_decoder_t* stream);

// Close and free a streaming decoder handle.
void ft8_stream_close(ft8_stream_decoder_t* stream);

#ifdef __cplusplus
}
#endif

#endif
