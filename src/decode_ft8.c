// Actual decoding code factored out by KA9Q June 2025
// File/directory handling code is now in main.c

#define _GNU_SOURCE 1
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <libgen.h>
#include <assert.h>
#include <time.h>

#include "ft8/decode.h"
#include "ft8/constants.h"
#include "ft8/ft8_config.h"

#include "common/wave.h"
#include "common/debug.h"
#include "fft/kiss_fftr.h"
#include "fft/kiss_fft.h"

#ifndef LOG_LEVEL
#define LOG_LEVEL LOG_DEBUG
#endif

const int kMin_score = FT8_MIN_SCORE; // Minimum sync score threshold for candidates
const int kMax_candidates = FT8_MAX_CANDIDATES; // for 12 kHz sample rate; scaled for other sample rates
const int kLDPC_iterations = FT8_LDPC_ITERATIONS;

// This used to be 50. We're now looking at some wider bandwidths *and* FT8 is pretty popular
// Making this bigger seems to only cost memory, which I now allocate from the heap, so what the hell
const int kMax_decoded_messages = FT8_MAX_DECODED_MSGS;

const int kFreq_osr = FT8_FREQ_OSR; // Frequency oversampling rate (bin subdivision)
const int kTime_osr = FT8_TIME_OSR; // Time oversampling rate (symbol subdivision)
static float hann_i(int i, int N)
{
    float x = sinf((float)M_PI * i / N);
    return x * x;
}

static float hamming_i(int i, int N)
{
    const float a0 = (float)25 / 46;
    const float a1 = 1 - a0;

    float x1 = cosf(2 * (float)M_PI * i / N);
    return a0 - a1 * x1;
}

static float blackman_i(int i, int N)
{
    const float alpha = 0.16f; // or 2860/18608
    const float a0 = (1 - alpha) / 2;
    const float a1 = 1.0f / 2;
    const float a2 = alpha / 2;

    float x1 = cosf(2 * (float)M_PI * i / N);
    float x2 = 2 * x1 * x1 - 1; // Use double angle formula

    return a0 - a1 * x1 + a2 * x2;
}

void waterfall_init(waterfall_t* me, int max_blocks, int num_bins, int time_osr, int freq_osr)
{
    size_t mag_size = max_blocks * time_osr * freq_osr * num_bins * sizeof(me->mag[0]);
    me->max_blocks = max_blocks;
    me->num_blocks = 0;
    me->num_bins = num_bins;
    me->time_osr = time_osr;
    me->freq_osr = freq_osr;
    me->block_stride = (time_osr * freq_osr * num_bins);
    me->mag = (uint8_t  *)malloc(mag_size);
    LOG(LOG_DEBUG, "Waterfall size = %zu\n", mag_size);
}

void waterfall_free(waterfall_t* me)
{
    free(me->mag);
}

/// Configuration options for FT4/FT8 monitor
typedef struct
{
    float f_min;             ///< Lower frequency bound for analysis
    float f_max;             ///< Upper frequency bound for analysis
    int sample_rate;         ///< Sample rate in Hertz
    int time_osr;            ///< Number of time subdivisions
    int freq_osr;            ///< Number of frequency subdivisions
    ftx_protocol_t protocol; ///< Protocol: FT4 or FT8
} monitor_config_t;

/// FT4/FT8 monitor object that manages DSP processing of incoming audio data
/// and prepares a waterfall object
typedef struct
{
    float symbol_period; ///< FT4/FT8 symbol period in seconds
    int block_size;      ///< Number of samples per symbol (block)
    int subblock_size;   ///< Analysis shift size (number of samples)
    int nfft;            ///< FFT size
    float fft_norm;      ///< FFT normalization factor
    float* window;       ///< Window function for STFT analysis (nfft samples)
    float* last_frame;   ///< Current STFT analysis frame (nfft samples)
    kiss_fft_scalar* timedata; ///< FFT input scratch buffer (nfft samples)
    kiss_fft_cpx* freqdata;    ///< FFT output scratch buffer (nfft/2+1 samples)
    waterfall_t wf;      ///< Waterfall object
    float max_mag;       ///< Maximum detected magnitude (debug stats)

    // KISS FFT housekeeping variables
    void* fft_work;        ///< Work area required by Kiss FFT
    kiss_fftr_cfg fft_cfg; ///< Kiss FFT housekeeping object
} monitor_t;

// Iinitialize a monitor_t structure based on the provided configuration (monitor_config_t). 
// It calculates DSP parameters, allocates memory for FFT processing and windowing, 
// sets up a waterfall display, and logs key initialization details, 
//preparing the monitor for signal processing tasks.
void monitor_init(monitor_t* me, const monitor_config_t* cfg)
{
    float slot_time = (cfg->protocol == PROTO_FT4) ? FT4_SLOT_TIME : FT8_SLOT_TIME;
    float symbol_period = (cfg->protocol == PROTO_FT4) ? FT4_SYMBOL_PERIOD : FT8_SYMBOL_PERIOD;
    // Compute DSP parameters that depend on the sample rate
    me->block_size = (int)(cfg->sample_rate * symbol_period); // samples corresponding to one FSK symbol
    me->subblock_size = me->block_size / cfg->time_osr;
    me->nfft = me->block_size * cfg->freq_osr;
    me->fft_norm = 2.0f / me->nfft;
    // const int len_window = 1.8f * me->block_size; // hand-picked and optimized

    me->window = (float *)malloc(me->nfft * sizeof(me->window[0]));
    for (int i = 0; i < me->nfft; ++i)
    {
        // window[i] = 1;
        me->window[i] = hann_i(i, me->nfft);
        // me->window[i] = blackman_i(i, me->nfft);
        // me->window[i] = hamming_i(i, me->nfft);
        // me->window[i] = (i < len_window) ? hann_i(i, len_window) : 0;
    }
    me->last_frame = (float *)malloc(me->nfft * sizeof(me->last_frame[0]));
    me->timedata = (kiss_fft_scalar*)malloc(me->nfft * sizeof(me->timedata[0]));
    me->freqdata = (kiss_fft_cpx*)malloc((me->nfft / 2 + 1) * sizeof(me->freqdata[0]));

    size_t fft_work_size;
    kiss_fftr_alloc(me->nfft, 0, 0, &fft_work_size);

    LOG(LOG_INFO, "Block size = %d\n", me->block_size);
    LOG(LOG_INFO, "Subblock size = %d\n", me->subblock_size);
    LOG(LOG_INFO, "N_FFT = %d\n", me->nfft);
    LOG(LOG_DEBUG, "FFT work area = %zu\n", fft_work_size);

    me->fft_work = malloc(fft_work_size);
    LOG(LOG_DEBUG, "init FFT for %d points work area allocated at %p\n", me->nfft, me->fft_work);
    me->fft_cfg = kiss_fftr_alloc(me->nfft, 0, me->fft_work, &fft_work_size);

    const int max_blocks = (int)(slot_time / symbol_period);
    const int num_bins = (int)(cfg->sample_rate * symbol_period / 2);
    waterfall_init(&me->wf, max_blocks, num_bins, cfg->time_osr, cfg->freq_osr);
    me->wf.protocol = cfg->protocol;
    me->symbol_period = symbol_period;

    me->max_mag = -120.0f;
}

void monitor_free(monitor_t* me)
{
    waterfall_free(&me->wf);
    free(me->fft_work);
  free(me->freqdata);
  free(me->timedata);
    free(me->last_frame);
    free(me->window);
}

// Compute FFT magnitudes (log wf) for a frame in the signal and update waterfall data
// Processes a frame of signal data by performing FFT-based analysis, 
// computing magnitudes in decibels, and updating the waterfall data structure with scaled values. 
// It handles time and frequency oversampling, applies a window function, 
// and ensures the results are clamped to an 8-bit range while tracking 
// the maximum magnitude observed.
void monitor_process(monitor_t* me, const float* frame)
{
    // Check if we can still store more waterfall data
    if (me->wf.num_blocks >= me->wf.max_blocks)
        return;
  if (me->timedata == NULL || me->freqdata == NULL)
    return;

    int offset = me->wf.num_blocks * me->wf.block_stride;
    int frame_pos = 0;

    // Loop over block subdivisions
    for (int time_sub = 0; time_sub < me->wf.time_osr; ++time_sub)
    {
      // Shift the new data into analysis frame
        for (int pos = 0; pos < me->nfft - me->subblock_size; ++pos)
        {
            me->last_frame[pos] = me->last_frame[pos + me->subblock_size];
        }
        for (int pos = me->nfft - me->subblock_size; pos < me->nfft; ++pos)
        {
            me->last_frame[pos] = frame[frame_pos];
            ++frame_pos;
        }

        // Compute windowed analysis frame
        for (int pos = 0; pos < me->nfft; ++pos)
        {
          me->timedata[pos] = me->fft_norm * me->window[pos] * me->last_frame[pos];
        }

        kiss_fftr(me->fft_cfg, me->timedata, me->freqdata);

        // Loop over two possible frequency bin offsets (for averaging)
        for (int freq_sub = 0; freq_sub < me->wf.freq_osr; ++freq_sub)
        {
            for (int bin = 0; bin < me->wf.num_bins; ++bin)
            {
                int src_bin = (bin * me->wf.freq_osr) + freq_sub;
                float mag2 = (me->freqdata[src_bin].i * me->freqdata[src_bin].i) + (me->freqdata[src_bin].r * me->freqdata[src_bin].r);
                float db = 10.0f * log10f(1E-12f + mag2);
                // Scale decibels to unsigned 8-bit range and clamp the value
                // Range 0-240 covers -120..0 dB in 0.5 dB steps
                int scaled = (int)(2 * db + 240);

                me->wf.mag[offset] = (scaled < 0) ? 0 : ((scaled > 255) ? 255 : scaled);
                ++offset;

                if (db > me->max_mag)
                    me->max_mag = db;
            }
        }
    }

    ++me->wf.num_blocks;
}

void monitor_reset(monitor_t* me)
{
    me->wf.num_blocks = 0;
    me->max_mag = 0;
}

// Used to sort messages by ascending frequency, and to push empty entries to end
int mcompare(void const *a, void const *b){
  message_t const *ma = *(message_t const **)a;
  message_t const *mb = *(message_t const **)b;
  // Null entries go to end of list
  if(ma == NULL && mb == NULL)
    return 0;
  else if(ma == NULL)
    return +1;
  else if(mb == NULL)
    return -1;
  if(ma->freq_hz > mb->freq_hz)
    return +1;
  else if(ma->freq_hz < mb->freq_hz)
    return -1;
  return 0;
}

static float mag_u8_to_power(uint8_t mag)
{
  // In monitor_process(): mag ~= 2*db + 240, where db = 10*log10(power)
  float db = 0.5f * ((float)mag - 240.0f);
  return powf(10.0f, db / 10.0f);
}

static float estimate_global_noise_power(const waterfall_t *wf)
{
  int hist[256] = {0};
  int total = wf->num_blocks * wf->block_stride;
  if (total <= 0)
    return 1e-12f;

  const uint8_t *p = wf->mag;
  for (int i = 0; i < total; ++i)
    ++hist[p[i]];

  // Use a lower percentile as robust noise-floor proxy, avoiding signal peaks.
  int target = (int)(0.35f * total);
  int acc = 0;
  int mag_floor = 0;
  for (int m = 0; m < 256; ++m)
  {
    acc += hist[m];
    if (acc >= target)
    {
      mag_floor = m;
      break;
    }
  }

  return mag_u8_to_power((uint8_t)mag_floor);
}

static float estimate_candidate_snr_db_2500(const waterfall_t *wf, const candidate_t *cand, const uint8_t *plain, float noise_power)
{
  float sum_sig = 0.0f;
  int n_sig = 0;
  int base = (cand->time_offset * wf->time_osr) + cand->time_sub;
  base = (base * wf->freq_osr) + cand->freq_sub;
  base = (base * wf->num_bins) + cand->freq_offset;

  if (wf->protocol == PROTO_FT8)
  {
    for (int k = 0; k < FT8_ND; ++k)
    {
      int sym_idx = k + ((k < 29) ? 7 : 14);
      int block_abs = cand->time_offset + sym_idx;
      if (block_abs < 0 || block_abs >= wf->num_blocks)
        continue;

      int bit_idx = 3 * k;
      int g = ((plain[bit_idx + 0] & 1) << 2) |
              ((plain[bit_idx + 1] & 1) << 1) |
              ((plain[bit_idx + 2] & 1) << 0);
      int tone = kFT8_Gray_map[g & 7];

      const uint8_t *p = wf->mag + base + (sym_idx * wf->block_stride);
      sum_sig += mag_u8_to_power(p[tone]);
      ++n_sig;
    }
  }
  else
  {
    for (int k = 0; k < FT4_ND; ++k)
    {
      int sym_idx = k + ((k < 29) ? 5 : ((k < 58) ? 9 : 13));
      int block_abs = cand->time_offset + sym_idx;
      if (block_abs < 0 || block_abs >= wf->num_blocks)
        continue;

      int bit_idx = 2 * k;
      int g = ((plain[bit_idx + 0] & 1) << 1) |
              ((plain[bit_idx + 1] & 1) << 0);
      int tone = kFT4_Gray_map[g & 3];

      const uint8_t *p = wf->mag + base + (sym_idx * wf->block_stride);
      sum_sig += mag_u8_to_power(p[tone]);
      ++n_sig;
    }
  }

  if (n_sig == 0)
    return -99.0f;

  float sig = sum_sig / n_sig;
  float noise = (noise_power > 0.0f) ? noise_power : 1e-12f;
  if (noise <= 0.0f)
    return -99.0f;

  // Convert local tone-group SNR to WSJT-style 2500 Hz reference.
  // Use per-tone bin bandwidth (~6.25 Hz for FT modes), then reference to 2500 Hz.
  float snr_bin_db = 10.0f * log10f(sig / noise + 1e-12f);
  float snr_2500_db = snr_bin_db - 10.0f * log10f(2500.0f / 6.25f);
  return snr_2500_db;
}

static double elapsed_ms(const struct timespec *t0, const struct timespec *t1)
{
  long sec = t1->tv_sec - t0->tv_sec;
  long nsec = t1->tv_nsec - t0->tv_nsec;
  return (double)sec * 1000.0 + (double)nsec / 1000000.0;
}

// ------------------------------------------------------------------------------------
// Step 1: Waterfall Accumulation
// ------------------------------------------------------------------------------------
void accumulate_waterfall(monitor_t* mon, const float* signal, int num_samples) {
    for (int frame_pos = 0; frame_pos + mon->block_size <= num_samples; frame_pos += mon->block_size) {
        monitor_process(mon, signal + frame_pos);
    }
}

// ------------------------------------------------------------------------------------
// Step 2: Candidate Search
// ------------------------------------------------------------------------------------
int find_candidates(const waterfall_t* wf, candidate_t* candidate_list, int candidate_size, int min_score) {
    return ft8_find_sync(wf, candidate_size, candidate_list, min_score);
}

static bool emitted_messages_contains(const message_t* seen_messages, int seen_count, const message_t* msg)
{
  for (int i = 0; i < seen_count; ++i)
  {
    if (seen_messages[i].hash == msg->hash && strcmp(seen_messages[i].text, msg->text) == 0)
      return true;
  }
  return false;
}

static void emitted_messages_add(message_t* seen_messages, int* seen_count, int seen_capacity, const message_t* msg)
{
  if (seen_messages == NULL || seen_count == NULL)
    return;
  if (*seen_count >= seen_capacity)
    return;
  seen_messages[*seen_count] = *msg;
  ++(*seen_count);
}

// ------------------------------------------------------------------------------------
// Step 3: Message Decoding
// ------------------------------------------------------------------------------------
int decode_candidates(const waterfall_t* wf, const candidate_t* candidate_list, int num_candidates,
                      message_t* decoded, int max_decoded, int ldpc_iterations, float noise_power,
                      struct tm const* tmp, double sec, float base_freq, bool is_ft8,
                      message_t* seen_messages, int* seen_count, int seen_capacity) {
    int num_decoded = 0;
    message_t **decoded_hashtable = calloc(sizeof(message_t *), max_decoded);
    
    for (int idx = 0; idx < num_candidates; ++idx) {
        const candidate_t* cand = &candidate_list[idx];
        if (cand->score < kMin_score)
            continue;

        float const freq_hz = (cand->freq_offset + (float)cand->freq_sub / wf->freq_osr) / (is_ft8 ? FT8_SYMBOL_PERIOD : FT4_SYMBOL_PERIOD);
        float const time_sec = (cand->time_offset + (float)cand->time_sub / wf->time_osr) * (is_ft8 ? FT8_SYMBOL_PERIOD : FT4_SYMBOL_PERIOD);

        message_t message = {0};
        decode_status_t status = {0};
        uint8_t plain174[FTX_LDPC_N];
        if (!ft8_decode(wf, cand, &message, ldpc_iterations, &status, plain174)) {
            if (status.ldpc_errors > 0) {
                // LOG(LOG_DEBUG, "LDPC decode: %d errors\n", status.ldpc_errors);
            } else if (status.crc_calculated != status.crc_extracted) {
                LOG(LOG_DEBUG, "CRC mismatch!\n");
            } else if (status.unpack_status != 0) {
                LOG(LOG_DEBUG, "Error while unpacking!\n");
            }
            continue;
        }

        message.freq_hz = freq_hz;
        message.time_sec = time_sec;
        message.score = cand->score;
        message.snr_raw_db = estimate_candidate_snr_db_2500(wf, cand, plain174, noise_power);
        {
            float snr = FT8_SNR_RAW_SCALE * message.snr_raw_db + FT8_SNR_SCORE_SCALE * (float)message.score + FT8_SNR_OFFSET;
            if (snr < FT8_SNR_MIN_DB)
                snr = FT8_SNR_MIN_DB;
            if (snr > FT8_SNR_MAX_DB)
                snr = FT8_SNR_MAX_DB;
            message.snr_db = snr;
        }

        // LOG(LOG_DEBUG, "Checking hash table with %4.1fs start offset / %4.1fHz offset [score %d]...\n", time_sec, freq_hz, cand->score);
        int idx_hash = message.hash % max_decoded;
        bool found_empty_slot = false;
        bool found_duplicate = false;
        do {
            if (decoded_hashtable[idx_hash] == NULL) {
                // LOG(LOG_DEBUG, "Found an empty slot\n");
                found_empty_slot = true;
            } else if ((decoded_hashtable[idx_hash]->hash == message.hash) && (0 == strcmp(decoded_hashtable[idx_hash]->text, message.text))) {
                // LOG(LOG_DEBUG, "Found a duplicate [%s]\n", message.text);
                found_duplicate = true;
            } else {
                LOG(LOG_ERROR, "Hash table clash!\n");
                idx_hash = (idx_hash + 1) % max_decoded;
            }
        } while (!found_empty_slot && !found_duplicate);

        if (found_empty_slot) {
            decoded[idx_hash] = message;
            decoded_hashtable[idx_hash] = &decoded[idx_hash];
            ++num_decoded;
        }
    }
    
    // Sort messages by frequency and push empty entries to end
    qsort(decoded_hashtable, max_decoded, sizeof(message_t *), mcompare);
    
    // Output messages
    double tbase = tmp->tm_sec;
    tbase = is_ft8 ? fmod(tbase, 15.0) : fmod(tbase, 7.5);
    tbase += sec;
    for(int i = 0; i < num_decoded; i++){
        message_t const *mp = decoded_hashtable[i];
        if(mp == NULL)
            continue;

      if (seen_messages != NULL && seen_count != NULL && seen_capacity > 0)
      {
        if (emitted_messages_contains(seen_messages, *seen_count, mp))
        {
          // LOG(LOG_DEBUG, "Suppressing duplicate message across stream phases [%s]\n", mp->text);
          continue;
        }
        emitted_messages_add(seen_messages, seen_count, seen_capacity, mp);
      }

      OUT("%4d/%02d/%02d %02d:%02d:%02d %3d %+4.2lf %'.1lf ~ %s\n",
            tmp->tm_year + 1900, tmp->tm_mon + 1, tmp->tm_mday,
            tmp->tm_hour, tmp->tm_min, tmp->tm_sec,
            (int)lroundf(mp->snr_db),
            tbase + mp->time_sec,
            1.0e6 * base_freq + mp->freq_hz,
            mp->text);
    }
    free(decoded_hashtable);
    return num_decoded;
}

struct process_stream
{
    monitor_t mon;
    monitor_config_t mon_cfg;
    float* carry;
    int carry_samples;
    bool finalized;
    bool is_ft8;
    float base_freq;
    struct tm utc;
    double utc_frac_sec;
    bool checkpoint_done;
    message_t* seen_messages;
    int seen_count;
    int seen_capacity;
};

  static int stream_decode_pass(process_stream_t* stream, const char* phase_name)
  {
    struct timespec t_cand0 = {0};
    struct timespec t_cand1 = {0};
    struct timespec t_dec1 = {0};

    float const noise_power = estimate_global_noise_power(&stream->mon.wf);
    int candidate_size = (stream->mon_cfg.f_max * kMax_candidates) / 3000;
    if (candidate_size < 1)
      candidate_size = 1;
    candidate_t candidate_list[candidate_size];

    clock_gettime(CLOCK_MONOTONIC, &t_cand0);
    int num_candidates = find_candidates(&stream->mon.wf, candidate_list, candidate_size, kMin_score);
    clock_gettime(CLOCK_MONOTONIC, &t_cand1);

    message_t* decoded = calloc((size_t)kMax_decoded_messages, sizeof(*decoded));
    if (decoded == NULL)
      return -1;

    int num_decoded = decode_candidates(&stream->mon.wf,
                      candidate_list,
                      num_candidates,
                      decoded,
                      kMax_decoded_messages,
                      stream->is_ft8 ? FT8_LDPC_ITERATIONS : 120,
                      noise_power,
                      &stream->utc,
                      stream->utc_frac_sec,
                      stream->base_freq,
                      stream->is_ft8,
                      stream->seen_messages,
                      &stream->seen_count,
                      stream->seen_capacity);
    clock_gettime(CLOCK_MONOTONIC, &t_dec1);

    LOG(LOG_INFO,
      "Streaming %s decode: %d candidates in %.3f ms, %d decoded in %.3f ms (emitted unique this slot: %d)\n",
      phase_name,
      num_candidates,
      elapsed_ms(&t_cand0, &t_cand1),
      num_decoded,
      elapsed_ms(&t_cand1, &t_dec1),
      stream->seen_count);

    free(decoded);
    return num_decoded;
  }

process_stream_t* process_stream_open(int sample_rate, bool is_ft8, float base_freq, struct tm const *tmp, double fsec)
{
    if (sample_rate <= 0 || tmp == NULL)
      return NULL;

    process_stream_t* stream = calloc(1, sizeof(*stream));
    if (stream == NULL)
      return NULL;

    stream->mon_cfg.f_min = 100;
    stream->mon_cfg.f_max = sample_rate / 2 - 500;
    stream->mon_cfg.sample_rate = sample_rate;
    stream->mon_cfg.time_osr = kTime_osr;
    stream->mon_cfg.freq_osr = kFreq_osr;
    stream->mon_cfg.protocol = is_ft8 ? PROTO_FT8 : PROTO_FT4;

    stream->is_ft8 = is_ft8;
    stream->base_freq = base_freq;
    stream->utc = *tmp;
    stream->utc_frac_sec = fsec;

    monitor_init(&stream->mon, &stream->mon_cfg);
    stream->carry = malloc((size_t)stream->mon.block_size * sizeof(stream->carry[0]));
    if (stream->carry == NULL)
    {
      monitor_free(&stream->mon);
      free(stream);
      return NULL;
    }

    stream->seen_capacity = 2 * kMax_decoded_messages;
    stream->seen_messages = calloc((size_t)stream->seen_capacity, sizeof(*stream->seen_messages));
    if (stream->seen_messages == NULL)
    {
      free(stream->carry);
      monitor_free(&stream->mon);
      free(stream);
      return NULL;
    }

    LOG(LOG_DEBUG, "Streaming waterfall open: block_size=%d, max_blocks=%d\n", stream->mon.block_size, stream->mon.wf.max_blocks);
    return stream;
}

int process_stream_append_float(process_stream_t* stream, const float* signal, int num_samples)
{
    if (stream == NULL || signal == NULL || num_samples < 0 || stream->finalized)
      return -1;
    if (num_samples == 0)
      return 0;

    int pos = 0;
    int blocks_processed = 0;
    const int checkpoint_blocks = stream->is_ft8 ? FT8_NN : FT4_NN;
    while (pos < num_samples)
    {
      int remaining = num_samples - pos;

      if (stream->carry_samples == 0 && remaining >= stream->mon.block_size)
      {
        monitor_process(&stream->mon, signal + pos);
        pos += stream->mon.block_size;
        ++blocks_processed;
        if (!stream->checkpoint_done && stream->mon.wf.num_blocks >= checkpoint_blocks)
        {
          LOG(LOG_INFO, "ckpoint reached: %d symbols accumulated\n", stream->mon.wf.num_blocks);
          if (stream_decode_pass(stream, "checkpoint") < 0)
            return -1;
          stream->checkpoint_done = true;
        }
        continue;
      }

      int needed = stream->mon.block_size - stream->carry_samples;
      int to_copy = (remaining < needed) ? remaining : needed;
      memcpy(stream->carry + stream->carry_samples, signal + pos, (size_t)to_copy * sizeof(stream->carry[0]));
      stream->carry_samples += to_copy;
      pos += to_copy;

      if (stream->carry_samples == stream->mon.block_size)
      {
        monitor_process(&stream->mon, stream->carry);
        stream->carry_samples = 0;
        ++blocks_processed;
        if (!stream->checkpoint_done && stream->mon.wf.num_blocks >= checkpoint_blocks)
        {
          LOG(LOG_INFO, "Streaming checkpoint reached: %d symbols accumulated\n", stream->mon.wf.num_blocks);
          if (stream_decode_pass(stream, "checkpoint") < 0)
            return -1;
          stream->checkpoint_done = true;
        }
      }
    }

    return blocks_processed;
}

int process_stream_finalize(process_stream_t* stream)
{
    if (stream == NULL)
      return -1;
    if (stream->finalized)
      return 0;

    stream->finalized = true;

    if (stream->carry_samples > 0)
    {
      LOG(LOG_INFO, "Streaming finalize: dropping %d tail samples (< %d block size)\n", stream->carry_samples, stream->mon.block_size);
    }

    return stream_decode_pass(stream, "final");
}

void process_stream_close(process_stream_t* stream)
{
    if (stream == NULL)
      return;
    monitor_free(&stream->mon);
    free(stream->carry);
    free(stream->seen_messages);
    free(stream);
}

// ------------------------------------------------------------------------------------
// ORIGINAL: Process a buffer already loaded from a file (kept for reference)
// Pass precise time of signal[0] (including fractional second) so we can reference to it
// ------------------------------------------------------------------------------------
int process_buffer_ori(float const *signal,int sample_rate, int num_samples, bool is_ft8, float base_freq, struct tm const *tmp, double sec){
  assert(signal != NULL && tmp != NULL);

  struct timespec t_wf0 = {0};
  struct timespec t_wf1 = {0};
  struct timespec t_dec0 = {0};
  struct timespec t_dec1 = {0};

  LOG(LOG_INFO, "Sample rate %d Hz, %d samples, %.3f seconds\n", sample_rate, num_samples, (double)num_samples / sample_rate);

  clock_gettime(CLOCK_MONOTONIC, &t_wf0);

  // Compute Waterfall accumulation (FFT)
  monitor_t mon = {0};
  monitor_config_t const mon_cfg = {
    .f_min = 100,
    .f_max = sample_rate/2 - 500, // allow room for the receiver filter rolloff
    .sample_rate = sample_rate,
    .time_osr = kTime_osr,
    .freq_osr = kFreq_osr,
    .protocol = is_ft8 ? PROTO_FT8 : PROTO_FT4
  };

  monitor_init(&mon, &mon_cfg);
  LOG(LOG_DEBUG, "Waterfall allocated %d blocks of size %d\n", mon.wf.max_blocks, mon.block_size);
  accumulate_waterfall(&mon, signal, num_samples);

 clock_gettime(CLOCK_MONOTONIC, &t_wf1);
  LOG(LOG_INFO, "Waterfall accumulation: %d blocks in %.3f ms\n", mon.wf.num_blocks, elapsed_ms(&t_wf0, &t_wf1));
  LOG(LOG_INFO, "Max magnitude: %.1f dB\n", mon.max_mag);
  
  float const noise_power = estimate_global_noise_power(&mon.wf);
  
  // Find top candidates by Costas sync score and localize them in time and frequency
  int const candidate_size = (mon_cfg.f_max * kMax_candidates) / 3000; // Scale by bandwidth relative to the original 3 kHz
  candidate_t candidate_list[candidate_size];
  int num_candidates = ft8_find_sync(&mon.wf, candidate_size, candidate_list, kMin_score);

  // Hash table for decoded messages (to check for duplicates)
  int num_decoded = 0;
  // Pointer to kMax_decoded_messages-element array of message_t structures
  message_t *decoded = calloc(sizeof(message_t), kMax_decoded_messages);
  // Pointer to kMax_decoded_messsages-element array of pointers to message_t structures
  message_t **decoded_hashtable = calloc(sizeof(message_t *), kMax_decoded_messages);

  clock_gettime(CLOCK_MONOTONIC, &t_dec0);
  LOG(LOG_DEBUG, "Found %d candidates with score from %d in %.3f milliseconds\n", num_candidates, kMin_score, elapsed_ms(&t_wf1, &t_dec0));

  // Go over candidates and attempt to decode messages
  for (int idx = 0; idx < num_candidates; ++idx){
      const candidate_t* cand = &candidate_list[idx];
      if (cand->score < kMin_score)
	      continue;

      float const freq_hz = (cand->freq_offset + (float)cand->freq_sub / mon.wf.freq_osr) / mon.symbol_period;
      float const time_sec = (cand->time_offset + (float)cand->time_sub / mon.wf.time_osr) * mon.symbol_period;

      message_t message = {0}; // Written by ft8_decode()
      decode_status_t status = {0}; // ditto
      uint8_t plain174[FTX_LDPC_N];
      if (!ft8_decode(&mon.wf, cand, &message, kLDPC_iterations, &status, plain174)){
	      // printf("000000 %3d %+4.2f %4.0f ~  ---\n", cand->score, time_sec, freq_hz);
        if (status.ldpc_errors > 0){
          LOG(LOG_DEBUG, "LDPC decode: %d errors\n", status.ldpc_errors);
        }else if (status.crc_calculated != status.crc_extracted){
          LOG(LOG_DEBUG, "CRC mismatch!\n");
        }else if (status.unpack_status != 0){
          LOG(LOG_DEBUG, "Error while unpacking!\n");
        }
        continue;
      }

      message.freq_hz = freq_hz; // Save so we can sort on it and display it
      message.time_sec = time_sec; // Time offset of start from nominal UTC :00/:15/:30/:45 or :00/:07.5/:15/...
      message.score = cand->score;
      message.snr_raw_db = estimate_candidate_snr_db_2500(&mon.wf, cand, plain174, noise_power);
      {
        // Affine calibration from raw estimator to WSJT-like displayed SNR.
        float snr = FT8_SNR_RAW_SCALE * message.snr_raw_db + FT8_SNR_SCORE_SCALE * (float)message.score + FT8_SNR_OFFSET;
        if (snr < FT8_SNR_MIN_DB)
          snr = FT8_SNR_MIN_DB;
        if (snr > FT8_SNR_MAX_DB)
          snr = FT8_SNR_MAX_DB;
        message.snr_db = snr;
      }

      LOG(LOG_DEBUG, "Checking hash table with %4.1fs start offset / %4.1fHz offset [score %d]...\n", time_sec, freq_hz, cand->score);
      int idx_hash = message.hash % kMax_decoded_messages;
      bool found_empty_slot = false;
      bool found_duplicate = false;
      do{
        if (decoded_hashtable[idx_hash] == NULL){
            LOG(LOG_DEBUG, "Found an empty slot\n");
            found_empty_slot = true;
        }else if ((decoded_hashtable[idx_hash]->hash == message.hash) && (0 == strcmp(decoded_hashtable[idx_hash]->text, message.text))){
            LOG(LOG_DEBUG, "Found a duplicate [%s]\n", message.text);
            found_duplicate = true;
        }else{
            LOG(LOG_DEBUG, "Hash table clash!\n");
            // Move on to check the next entry in hash table
            idx_hash = (idx_hash + 1) % kMax_decoded_messages;
        }
      } while (!found_empty_slot && !found_duplicate);

      if (found_empty_slot){
        // Fill the empty hashtable slot
        decoded[idx_hash] = message;
        decoded_hashtable[idx_hash] = &decoded[idx_hash];
        ++num_decoded;
      }
  }
  
  clock_gettime(CLOCK_MONOTONIC, &t_dec1);
  LOG(LOG_INFO, "On %d candidates, decoded %d messages in %.3f ms\n", num_candidates, num_decoded, elapsed_ms(&t_dec0, &t_dec1));
  LOG(LOG_INFO, "Decoded %d messages\n", num_decoded);
  
  // Decoded messages are spread throughout hash table, so sort the whole thing including null entries
  qsort(decoded_hashtable, kMax_decoded_messages, sizeof *decoded_hashtable, mcompare);
  // Empty entries sorted to top, so first num_decoded elements of decoded_hashtable are valid
  double tbase = tmp->tm_sec; // Full seconds and fraction in minute, should be just above (not below) period multiple
  tbase = is_ft8 ? fmod(tbase,15.0) : fmod(tbase,7.5); // seconds after start of cycle (0/15/30/45 or 0/7.5/15/etc)
  tbase += sec; // sec could be negative, so add it only now

  for(int i=0; i < num_decoded; i++){
    message_t const *mp = decoded_hashtable[i];
    if(mp == NULL)
      continue; // Shouldn't happen

    OUT("%4d/%02d/%02d %02d:%02d:%02d %3d %+4.2lf %'.1lf ~ %s\n",
	    tmp->tm_year + 1900,
	    tmp->tm_mon + 1,
	    tmp->tm_mday,
	    tmp->tm_hour,
	    tmp->tm_min,
	    tmp->tm_sec,
      (int)lroundf(mp->snr_db),
	    tbase + mp->time_sec,
	    1.0e6 * base_freq + mp->freq_hz,
      mp->text);
  }
  free(decoded);
  free(decoded_hashtable);

  monitor_free(&mon);
  return 0; // Caller frees signal
}

// ------------------------------------------------------------------------------------
// REFACTORED: Process a buffer using three discrete phases
// Phase 1: Waterfall Accumulation (~14.8ms for typical signal)
// Phase 2: Candidate Search (~22ms for typical signal)  
// Phase 3: Message Decoding (~55ms for typical signal)
// ------------------------------------------------------------------------------------
int process_buffer(float const *signal, int sample_rate, int num_samples, bool is_ft8, float base_freq, struct tm const *tmp, double sec){
  assert(signal != NULL && tmp != NULL);

  struct timespec t_phase0 = {0};
  struct timespec t_phase1 = {0};
  struct timespec t_phase2 = {0};
  struct timespec t_phase3 = {0};

  LOG(LOG_INFO, "Sample rate %d Hz, %d samples, %.3f seconds\n", sample_rate, num_samples, (double)num_samples / sample_rate);

  clock_gettime(CLOCK_MONOTONIC, &t_phase0);

  // Initialize monitor
  monitor_t mon = {0};
  monitor_config_t const mon_cfg = {
    .f_min = 100,
    .f_max = sample_rate/2 - 500,
    .sample_rate = sample_rate,
    .time_osr = kTime_osr,
    .freq_osr = kFreq_osr,
    .protocol = is_ft8 ? PROTO_FT8 : PROTO_FT4
  };
  monitor_init(&mon, &mon_cfg);
  LOG(LOG_DEBUG, "Waterfall allocated %d blocks of size %d\n", mon.wf.max_blocks, mon.block_size);

  // Phase 1: Waterfall Accumulation
  accumulate_waterfall(&mon, signal, num_samples);
  clock_gettime(CLOCK_MONOTONIC, &t_phase1);
  LOG(LOG_INFO, "Phase 1 - Waterfall accumulation: %d blocks in %.3f ms\n", mon.wf.num_blocks, elapsed_ms(&t_phase0, &t_phase1));
  LOG(LOG_INFO, "Max magnitude: %.1f dB\n", mon.max_mag);
  
  float const noise_power = estimate_global_noise_power(&mon.wf);
  
  // Phase 2: Candidate Search
  int const candidate_size = (mon_cfg.f_max * kMax_candidates) / 3000;
  candidate_t candidate_list[candidate_size];
  int num_candidates = find_candidates(&mon.wf, candidate_list, candidate_size, kMin_score);
  clock_gettime(CLOCK_MONOTONIC, &t_phase2);
  LOG(LOG_INFO, "Phase 2 - Candidate search: Found %d candidates in %.3f ms\n", num_candidates, elapsed_ms(&t_phase1, &t_phase2));

  // Phase 3: Message Decoding
  message_t *decoded = calloc(sizeof(message_t), kMax_decoded_messages);
  int num_decoded = decode_candidates(&mon.wf, candidate_list, num_candidates, decoded, kMax_decoded_messages,
                                      is_ft8 ? FT8_LDPC_ITERATIONS : 120, noise_power, tmp, sec, base_freq, is_ft8,
                                      NULL, NULL, 0);
  clock_gettime(CLOCK_MONOTONIC, &t_phase3);
  LOG(LOG_INFO, "Phase 3 - Message decoding: Decoded %d messages in %.3f ms\n", num_decoded, elapsed_ms(&t_phase2, &t_phase3));

  free(decoded);
  monitor_free(&mon);
  return 0;
}
