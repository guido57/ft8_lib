#include "ft8_decode/decoder_api.h"

#include <stdlib.h>

#include "common/wave.h"

struct ft8_stream_decoder {
    process_stream_t* core;
};

ft8_stream_decoder_t* ft8_stream_open(int sample_rate, const ft8_decode_context_t* ctx)
{
    if (ctx == NULL)
        return NULL;

    ft8_stream_decoder_t* stream = calloc(1, sizeof(*stream));
    if (stream == NULL)
        return NULL;

    stream->core = process_stream_open(sample_rate,
                                       ctx->is_ft8,
                                       ctx->base_freq_mhz,
                                       &ctx->utc,
                                       ctx->utc_frac_sec);
    if (stream->core == NULL)
    {
        free(stream);
        return NULL;
    }

    return stream;
}

int ft8_stream_append_i16(ft8_stream_decoder_t* stream, const int16_t* signal, int num_samples)
{
    if (stream == NULL || signal == NULL || num_samples < 0)
        return -1;
    if (num_samples == 0)
        return 0;

    float* tmp = malloc((size_t)num_samples * sizeof(*tmp));
    if (tmp == NULL)
        return -1;

    for (int i = 0; i < num_samples; ++i)
    {
        tmp[i] = signal[i] * (1.0f / 32768.0f);
    }

    int rc = process_stream_append_float(stream->core, tmp, num_samples);
    free(tmp);
    return rc;
}

int ft8_stream_finalize(ft8_stream_decoder_t* stream)
{
    if (stream == NULL)
        return -1;
    return process_stream_finalize(stream->core);
}

void ft8_stream_close(ft8_stream_decoder_t* stream)
{
    if (stream == NULL)
        return;
    process_stream_close(stream->core);
    free(stream);
}
