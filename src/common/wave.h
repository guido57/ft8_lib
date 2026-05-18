#ifndef _INCLUDE_WAVE_H_
#define _INCLUDE_WAVE_H_

#include <stdbool.h>
#include <time.h>

#ifdef __cplusplus
extern "C"
{
#endif

  // Save signal in floating point format (-1 .. +1) as a WAVE file using 16-bit signed integers.
  void save_wav(const float* signal, int num_samples, int sample_rate, const char* path);

  // Load signal in floating point format (-1 .. +1) as a WAVE file using 16-bit signed integers.
  // Now mallocs signal array, places in *signal, caller must free
  int load_wav(float** signal, int* num_samples, int *num_channels, int* sample_rate, const char* path,int fd);

  // Opaque streaming decode state used by decoder_api wrappers.
  typedef struct process_stream process_stream_t;

  // Open/append/finalize/close streaming decode pipeline.
  process_stream_t* process_stream_open(int sample_rate, bool is_ft8, float base_freq, struct tm const *tmp, double fsec);
  int process_stream_append_float(process_stream_t* stream, const float* signal, int num_samples);
  int process_stream_finalize(process_stream_t* stream);
  void process_stream_close(process_stream_t* stream);

#ifdef __cplusplus
}
#endif

#endif // _INCLUDE_WAVE_H_
