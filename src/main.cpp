// ft8_lib cross-platform entrypoint
// - ESP32 Arduino: scans LittleFS and decodes .wav files
// - Linux host: decodes a wav path provided as argv[1]
//
// Flash the filesystem image first:
//   pio run --target uploadfs --environment esp32s3
//
// Then flash the firmware:
//   pio run --target upload --environment esp32s3

#include <time.h>
#include <stdint.h>

#if defined(ARDUINO_ARCH_ESP32)
#include <Arduino.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <sys/time.h>
#else
#include <cstdlib>   // malloc, free
#include <cstring>   // memcmp
#include <math.h>
#include <errno.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#endif

#include "ft8_decode/decoder_api.h"
#include "ft8_decode/decode.h"
#include "common/debug.h"
#include "secrets.h"
#include "ft8_consumer_module.h"

#if defined(ARDUINO_ARCH_ESP32)

// ============================================================================
// ESP32 Arduino Build
// ============================================================================

static float* alloc_sample_buffer(size_t count)
{
    size_t bytes = count * sizeof(float);
    // Prefer PSRAM for large FT8 slot buffers; internal DRAM is usually too small.
    if (psramFound()) {
        float* p = (float*)heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (p) {
            return p;
        }
    }
    return (float*)malloc(bytes);
}

static void get_utc_now(struct tm* utc, double* frac_sec)
{
    struct timeval tv = {0};
    gettimeofday(&tv, nullptr);
    time_t sec = tv.tv_sec;
    gmtime_r(&sec, utc);
    *frac_sec = (double)tv.tv_usec / 1000000.0;
}

static void format_utc_hhmmss_mmm(char* buf, size_t buf_len, const struct timeval& tv)
{
    struct tm utc = {0};
    time_t sec = tv.tv_sec;
    gmtime_r(&sec, &utc);
    snprintf(buf, buf_len, "%02d:%02d:%02d.%03d",
             utc.tm_hour,
             utc.tm_min,
             utc.tm_sec,
             (int)(tv.tv_usec / 1000));
}

static double seconds_to_next_ft8_slot(const struct tm* utc, double frac_sec)
{
    double sec_in_min = (double)utc->tm_sec + frac_sec;
    int quarter = (int)(sec_in_min / 15.0);
    double rem = sec_in_min - 15.0 * (double)quarter;
    if (rem < 1e-6)
        return 0.0;
    return 15.0 - rem;
}

static void sleep_seconds(double sec)
{
    if (sec <= 0.0)
        return;

    uint32_t ms = (uint32_t)(sec * 1000.0);
    if (ms > 0)
    {
        delay(ms);
    }

    double rem = sec - ((double)ms / 1000.0);
    if (rem > 0.0)
    {
        uint32_t us = (uint32_t)(rem * 1000000.0);
        if (us > 0)
            delayMicroseconds(us);
    }
}

static void sleep_until_mono_us(int64_t deadline_us)
{
    for (;;)
    {
        int64_t now_us = esp_timer_get_time();
        int64_t remain_us = deadline_us - now_us;
        if (remain_us <= 0)
            return;

        if (remain_us > 2000)
        {
            delay((uint32_t)(remain_us / 1000));
        }
        else
        {
            delayMicroseconds((uint32_t)remain_us);
        }
    }
}

static bool init_wifi()
{
    Serial.println("[ft8] Connecting to WiFi...");
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.persistent(false);

    const uint8_t bssid[6] = SECRET_BSSID;
    WiFi.begin(SECRET_SSID, SECRET_PASS, 0, bssid, true);

    const int max_attempts = 300; // 30 seconds at 100 ms cadence
    int attempt = 0;
    while (WiFi.status() != WL_CONNECTED && attempt < max_attempts)
    {
        delay(100);
        ++attempt;
        if ((attempt % 10) == 0)
        {
            Serial.printf("[ft8] WiFi connect in progress... (%d%%)\n", (attempt * 100) / max_attempts);
        }
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.printf("[ft8] WiFi connected, IP=%s, RSSI=%d dBm\n",
                      WiFi.localIP().toString().c_str(),
                      WiFi.RSSI());
        return true;
    }

    Serial.printf("[ft8] WARNING: WiFi connect failed (status=%d), continuing without network\n", (int)WiFi.status());
    return false;
}

static void init_ntp()
{
    Serial.println("[ft8] Initializing NTP for wall-clock synchronization");
    
    // Configure timezone: UTC+0, no DST
    // configTime(gmtOffset_sec, daylightOffset_sec, server1, server2, server3)
    configTime(0, 0, "pool.ntp.org", "time.nist.gov", "time.google.com");
    
    // Wait for NTP sync to complete (up to 15 seconds)
    Serial.println("[ft8] Waiting for NTP synchronization...");
    time_t now = time(nullptr);
    int max_attempts = 150;  // 15 seconds with 100ms delay
    int attempt = 0;
    
    while (now < 24 * 3600 && attempt < max_attempts) {
        delay(100);
        now = time(nullptr);
        attempt++;
        
        if (attempt % 10 == 0) {
            Serial.printf("[ft8] NTP sync in progress... (%d%%)\n", (attempt * 100) / max_attempts);
        }
    }
    
    if (now > 24 * 3600) {
        // Time is synced (unix timestamp is beyond 1970-01-02)
        struct tm utc = {0};
        gmtime_r(&now, &utc);
        Serial.printf("[ft8] NTP synchronized: %04d-%02d-%02d %02d:%02d:%02d UTC\n",
                      utc.tm_year + 1900, utc.tm_mon + 1, utc.tm_mday,
                      utc.tm_hour, utc.tm_min, utc.tm_sec);
    } else {
        Serial.println("[ft8] WARNING: NTP sync timeout, using system time (may be inaccurate)");
    }
}

static constexpr int kMaxWavFiles = 64;
static constexpr int kMaxPathLen = 96;
static constexpr int kSampleQueueDepth = 32768;
static constexpr int kProducerPrefetchSamples = 32768;
static constexpr int kProducerStartupPrefillSamples = 2048;
static constexpr int kProducerOutputSampleRate = 8000;

typedef struct {
    File file;
    uint16_t num_channels;
    uint32_t sample_rate;
    uint32_t data_start;
    uint32_t data_size;
    uint32_t num_samples;
    uint32_t sample_index;
    bool valid;
} wav_stream_t;

extern "C" void ft8_on_message_decoded(const char* phase,
                                         const struct tm* utc,
                                         double tbase_sec,
                                         float base_freq_mhz,
                                         const message_t* msg)
{
    if (phase == nullptr || utc == nullptr || msg == nullptr) {
        return;
    }

    // Emit callback notifications for final pass only.
    if (strcmp(phase, "final") != 0) {
        return;
    }

    Serial.printf("[cb] %04d/%02d/%02d %02d:%02d:%02d %3d %+4.2lf %'.1lf ~ %s\n",
                  utc->tm_year + 1900,
                  utc->tm_mon + 1,
                  utc->tm_mday,
                  utc->tm_hour,
                  utc->tm_min,
                  utc->tm_sec,
                  (int)lroundf(msg->snr_db),
                  tbase_sec + msg->time_sec,
                  1.0e6 * base_freq_mhz + msg->freq_hz,
                  msg->text);
}

typedef struct {
    int16_t* data;
    int capacity;
    int read_idx;
    int write_idx;
    int count;
} sample_ring_t;

static bool sample_ring_init(sample_ring_t* ring, int capacity)
{
    ring->data = (int16_t*)malloc((size_t)capacity * sizeof(int16_t));
    if (ring->data == nullptr)
        return false;
    ring->capacity = capacity;
    ring->read_idx = 0;
    ring->write_idx = 0;
    ring->count = 0;
    return true;
}

static void sample_ring_free(sample_ring_t* ring)
{
    if (ring->data != nullptr)
        free(ring->data);
    ring->data = nullptr;
    ring->capacity = 0;
    ring->read_idx = 0;
    ring->write_idx = 0;
    ring->count = 0;
}

static bool sample_ring_push(sample_ring_t* ring, int16_t sample)
{
    if (ring->count >= ring->capacity)
        return false;
    ring->data[ring->write_idx] = sample;
    ring->write_idx = (ring->write_idx + 1) % ring->capacity;
    ++ring->count;
    return true;
}

static bool sample_ring_pop(sample_ring_t* ring, int16_t* out_sample)
{
    if (ring->count <= 0)
        return false;
    *out_sample = ring->data[ring->read_idx];
    ring->read_idx = (ring->read_idx + 1) % ring->capacity;
    --ring->count;
    return true;
}

static bool wav_stream_rewind(wav_stream_t* ws)
{
    if (!ws->file.seek(ws->data_start))
        return false;
    ws->sample_index = 0;
    return true;
}

static bool wav_stream_open(const char* path, wav_stream_t* ws)
{
    memset(ws, 0, sizeof(*ws));
    ws->file = LittleFS.open(path, "r");
    if (!ws->file) {
        Serial.printf("[ft8] Cannot open %s\n", path);
        return false;
    }

    // Read and parse WAV header (RIFF container)
    uint8_t riff_header[12];
    if (ws->file.read(riff_header, 12) != 12 || memcmp(riff_header, "RIFF", 4) != 0 || memcmp(riff_header + 8, "WAVE", 4) != 0) {
        Serial.println("[ft8] Not a valid WAVE file");
        ws->file.close();
        return false;
    }

    // Read chunks until we find "fmt " and "data"
    uint8_t chunk_header[8];
    uint32_t chunk_size = 0;
    uint16_t audio_format = 0;
    uint16_t bits_per_sample = 0;
    bool found_fmt = false, found_data = false;

    while (ws->file.read(chunk_header, 8) == 8) {
        chunk_size = (chunk_header[7] << 24) | (chunk_header[6] << 16) | (chunk_header[5] << 8) | chunk_header[4];

        if (memcmp(chunk_header, "fmt ", 4) == 0) {
            uint8_t fmt_data[16];
            if (ws->file.read(fmt_data, 16) != 16) break;
            audio_format = (fmt_data[1] << 8) | fmt_data[0];
            ws->num_channels = (fmt_data[3] << 8) | fmt_data[2];
            ws->sample_rate = (fmt_data[7] << 24) | (fmt_data[6] << 16) | (fmt_data[5] << 8) | fmt_data[4];
            bits_per_sample = (fmt_data[15] << 8) | fmt_data[14];
            found_fmt = true;
            if (chunk_size > 16) ws->file.seek(ws->file.position() + chunk_size - 16);
        } else if (memcmp(chunk_header, "data", 4) == 0) {
            ws->data_start = ws->file.position();
            ws->data_size = chunk_size;
            found_data = true;
            break;
        } else {
            ws->file.seek(ws->file.position() + chunk_size);
        }
    }

    if (!found_fmt || !found_data) {
        Serial.println("[ft8] Missing fmt or data chunk");
        ws->file.close();
        return false;
    }

    if (audio_format != 1 || bits_per_sample != 16) {
        Serial.printf("[ft8] Unsupported format: %u-bit, format %u\n", bits_per_sample, audio_format);
        ws->file.close();
        return false;
    }

    if (ws->num_channels == 0) {
        Serial.println("[ft8] Invalid channel count");
        ws->file.close();
        return false;
    }

    ws->num_samples = ws->data_size / ((uint32_t)ws->num_channels * 2U);
    ws->sample_index = 0;
    ws->valid = wav_stream_rewind(ws);
    if (!ws->valid) {
        Serial.println("[ft8] Failed to seek WAV data start");
        ws->file.close();
        return false;
    }

    return true;
}

static void wav_stream_close(wav_stream_t* ws)
{
    if (ws->file)
        ws->file.close();
    ws->valid = false;
}

static bool wav_stream_read_sample(wav_stream_t* ws, int16_t* out_sample)
{
    if (!ws->valid)
        return false;

    if (ws->sample_index >= ws->num_samples) {
        if (!wav_stream_rewind(ws))
            return false;
    }

    int16_t raw_sample = 0;
    if (ws->file.read((uint8_t*)&raw_sample, 2) != 2) {
        if (!wav_stream_rewind(ws))
            return false;
        if (ws->file.read((uint8_t*)&raw_sample, 2) != 2)
            return false;
    }

    for (uint16_t ch = 1; ch < ws->num_channels; ++ch) {
        int16_t discard = 0;
        if (ws->file.read((uint8_t*)&discard, 2) != 2)
            return false;
    }

    ++ws->sample_index;
    *out_sample = raw_sample;
    return true;
}

static bool fill_sample_ring_from_wav(sample_ring_t* ring, wav_stream_t* ws, int target_fill)
{
    if (target_fill > ring->capacity)
        target_fill = ring->capacity;

    while (ring->count < target_fill) {
        int16_t sample = 0;
        if (!wav_stream_read_sample(ws, &sample))
            return false;
        if (!sample_ring_push(ring, sample))
            return false;
    }
    return true;
}

static int collect_wav_paths(char paths[kMaxWavFiles][kMaxPathLen])
{
    int count = 0;
    File root = LittleFS.open("/");
    File entry;
    while ((entry = root.openNextFile()) && count < kMaxWavFiles) {
        String name = String("/") + entry.name();
        entry.close();
        if (name.endsWith(".wav") || name.endsWith(".WAV")) {
            snprintf(paths[count], kMaxPathLen, "%s", name.c_str());
            ++count;
        }
    }
    return count;
}

static void sample_producer_task(void* /*arg*/)
{
    if (!LittleFS.begin(true)) {
        Serial.println("[ft8] LittleFS mount failed - reflash filesystem");
        vTaskDelete(nullptr);
        return;
    }

    char wav_paths[kMaxWavFiles][kMaxPathLen] = {{0}};
    int wav_count = collect_wav_paths(wav_paths);
    Serial.printf("[ft8] Found %d wav files in LittleFS\n", wav_count);
    if (wav_count <= 0) {
        vTaskDelete(nullptr);
        return;
    }

    struct tm slot_start_tm = {0};
    double slot_start_frac = 0.0;
    time_t slot_start_epoch = 0;
    int64_t slot_start_deadline_us = 0;

    for (int idx = 0; idx < wav_count; ++idx) {
        wav_stream_t wav_local = {};
        wav_stream_t* wav = &wav_local;

        if (!wav_stream_open(wav_paths[idx], &wav_local)) {
            continue;
        }

        const int source_sample_rate = (int)wav->sample_rate;
        const int output_sample_rate = kProducerOutputSampleRate;
        const int slot_samples = (int)(15.0f * output_sample_rate + 0.5f);
        if (source_sample_rate <= 0 || output_sample_rate <= 0) {
            Serial.printf("[ft8] Invalid sample rate for %s (src=%d, out=%d)\n",
                          wav_paths[idx],
                          source_sample_rate,
                          output_sample_rate);
            wav_stream_close(wav);
            continue;
        }

        if (idx == 0) {
            struct timeval tv_now = {0};
            gettimeofday(&tv_now, nullptr);
            double epoch_now = (double)tv_now.tv_sec + (double)tv_now.tv_usec / 1000000.0;
            long slot_index = (long)(epoch_now / 15.0);
            double slot_epoch_d = (double)slot_index * 15.0;
            if (epoch_now - slot_epoch_d > 1e-6) {
                slot_epoch_d += 15.0;
            }

            double wait_sec = slot_epoch_d - epoch_now;
            if (wait_sec > 0.0) {
                Serial.printf("[ft8] waiting %.3f s for initial slot boundary 00/15/30/45\n", wait_sec);
                sleep_seconds(wait_sec);
            }

            slot_start_epoch = (time_t)slot_epoch_d;
            slot_start_deadline_us = esp_timer_get_time();
        }

        gmtime_r(&slot_start_epoch, &slot_start_tm);
        slot_start_frac = 0.0;

        sample_ring_t prefetch = {};
        int ring_capacity = kProducerPrefetchSamples;
        if (ring_capacity > slot_samples)
            ring_capacity = slot_samples;

        if (!sample_ring_init(&prefetch, ring_capacity)) {
            Serial.println("[ft8] Failed to allocate producer prefetch ring");
            wav_stream_close(wav);
            continue;
        }

        int startup_prefill = kProducerStartupPrefillSamples;
        if (startup_prefill > prefetch.capacity)
            startup_prefill = prefetch.capacity;
        if (!fill_sample_ring_from_wav(&prefetch, wav, startup_prefill)) {
            Serial.printf("[ft8] Failed to prefill samples for %s\n", wav_paths[idx]);
            sample_ring_free(&prefetch);
            wav_stream_close(wav);
            continue;
        }

        if (slot_start_deadline_us > 0) {
            sleep_until_mono_us(slot_start_deadline_us);
        }

        struct timeval producer_start_tv = {0};
        gettimeofday(&producer_start_tv, nullptr);
        char producer_start_buf[32] = {0};
        format_utc_hhmmss_mmm(producer_start_buf, sizeof(producer_start_buf), producer_start_tv);
        const int64_t producer_start_us = esp_timer_get_time();

        Serial.printf("[ft8] Producer slot #%d @ %s (target %d Hz)\n",
                      idx + 1,
                      producer_start_buf,
                      output_sample_rate);

        const int64_t slot_end_deadline_us = slot_start_deadline_us + 15000000LL;
        int sent_samples = 0;
        int source_samples_consumed = 0;
        int16_t current_sample = 0;
        bool has_current_sample = false;
        int downsample_phase = 0;

        auto pop_source_sample = [&](int16_t* out_sample, int out_i) -> bool {
            if (prefetch.count <= (prefetch.capacity / 2)) {
                if (!fill_sample_ring_from_wav(&prefetch, wav, prefetch.capacity)) {
                    Serial.printf("[ft8] Failed to refill samples for %s at output sample %d\n", wav_paths[idx], out_i);
                    return false;
                }
            }

            if (!sample_ring_pop(&prefetch, out_sample)) {
                Serial.printf("[ft8] Producer prefetch underrun for %s at output sample %d\n", wav_paths[idx], out_i);
                return false;
            }
            return true;
        };

        // Real-time resampling for 12 kHz inputs:
        // 1) duplicate each source sample to form a 24 kHz virtual stream
        // 2) emit 1 sample out of every 3 virtual samples to produce 8 kHz
        while (sent_samples < slot_samples) {
            if (esp_timer_get_time() >= slot_end_deadline_us) {
                Serial.printf("[ft8] %s slot cutoff at boundary: sent %d/%d samples\n",
                              wav_paths[idx],
                              sent_samples,
                              slot_samples);
                break;
            }

            if (!has_current_sample) {
                if (!pop_source_sample(&current_sample, source_samples_consumed)) {
                    break;
                }
                has_current_sample = true;
            }

            for (int dup = 0; dup < 2 && sent_samples < slot_samples; ++dup) {
                const int16_t virtual_sample = current_sample;

                if (downsample_phase == 0) {
                    ft8_consumer_module_enqueue_i16(&virtual_sample, 1, portMAX_DELAY);
                    ++sent_samples;

                    if (((sent_samples & 63) == 63) || (sent_samples == slot_samples)) {
                        int64_t target_us = slot_start_deadline_us + ((int64_t)sent_samples * 1000000LL) / (int64_t)output_sample_rate;
                        if (target_us < slot_end_deadline_us) {
                            sleep_until_mono_us(target_us);
                        }
                    }
                }

                downsample_phase = (downsample_phase + 1) % 3;
            }

            ++source_samples_consumed;
            has_current_sample = false;
        }

        const int64_t producer_end_us = esp_timer_get_time();
        const double producer_elapsed_sec = (double)(producer_end_us - producer_start_us) / 1000000.0;
        if (producer_elapsed_sec > 0.0) {
            struct timeval producer_end_tv = {0};
            gettimeofday(&producer_end_tv, nullptr);
            char producer_end_buf[32] = {0};
            format_utc_hhmmss_mmm(producer_end_buf, sizeof(producer_end_buf), producer_end_tv);
            const double measured_rate = (double)sent_samples / producer_elapsed_sec;
            Serial.printf("[ft8] Producer slot #%d done @ %s: %d samples in %.3f s = %.1f Hz (target %d Hz)\n",
                          idx + 1,
                          producer_end_buf,
                          sent_samples,
                          producer_elapsed_sec,
                          measured_rate,
                          output_sample_rate);
        }

        if (sent_samples < slot_samples) {
            Serial.printf("[ft8] %s dropped %d trailing samples to keep slot boundary\n",
                          wav_paths[idx],
                          slot_samples - sent_samples);
        }

        sample_ring_free(&prefetch);
        wav_stream_close(wav);

        slot_start_epoch += 15;
        slot_start_deadline_us += 15000000LL;
        gmtime_r(&slot_start_epoch, &slot_start_tm);
        slot_start_frac = 0.0;
    }

    Serial.println("[ft8] Producer completed all wav files");
    vTaskDelete(nullptr);
}

void setup()
{
    Serial.begin(115200);
    while (!Serial) delay(10);
    delay(1500);
    
    Serial.println("\n[ft8] FT8 decoder starting");
    Serial.printf("[ft8] Free heap: %lu bytes\n", (unsigned long)ESP.getFreeHeap());
    Serial.printf("[ft8] PSRAM found: %s, free PSRAM: %lu bytes\n", 
                  psramFound() ? "yes" : "no", (unsigned long)ESP.getFreePsram());

    init_wifi();
    
    // Initialize NTP for accurate wall-clock time
    init_ntp();

    // Configure and initialize the FT8 consumer module
    ft8_consumer_module_config_t consumer_cfg = {
        .sample_rate = kProducerOutputSampleRate,
        .base_freq_mhz = 14.074f,
        .sample_queue_depth = 2048, // kSampleQueueDepth,
        .finalize_queue_depth = 4,
        .append_batch_size = 256,
        .consumer_task_stack = 8192, // 12288,
        .finalize_task_stack = 8192, // 24576,
        .consumer_task_priority = 2,
        .finalize_task_priority = 1,
        .consumer_task_core = 0, // APP_CPU_NUM,
        .finalize_task_core = 0, // APP_CPU_NUM,
    };

    if (!ft8_consumer_module_init(&consumer_cfg)) {
        Serial.println("[ft8] Failed to initialize consumer module");
        return;
    }

    if (!ft8_consumer_module_start()) {
        Serial.println("[ft8] Failed to start consumer module");
        return;
    }

    BaseType_t rc_producer = xTaskCreatePinnedToCore(
        sample_producer_task,
        "ft8_producer",
        32768,
        nullptr,
        1,
        nullptr,
        APP_CPU_NUM
    );

    if (rc_producer != pdPASS) {
        Serial.printf("[ft8] Failed to create producer task (rc=%ld)\n", (long)rc_producer);
    }
}

void loop()
{
    delay(10000);
}

#else

// ============================================================================
// Native Linux Build
// ============================================================================

#include "common/wave.h"

static float* alloc_sample_buffer(size_t count)
{
    size_t bytes = count * sizeof(float);
    return (float*)malloc(bytes);
}

static void get_utc_now(struct tm* utc, double* frac_sec)
{
    struct timespec rt = {0};
    clock_gettime(CLOCK_REALTIME, &rt);
    time_t sec = rt.tv_sec;
    gmtime_r(&sec, utc);
    *frac_sec = (double)rt.tv_nsec / 1000000000.0;
}

static double seconds_to_next_ft8_slot(const struct tm* utc, double frac_sec)
{
    double sec_in_min = (double)utc->tm_sec + frac_sec;
    double rem = fmod(sec_in_min, 15.0);
    if (rem < 0.0)
        rem += 15.0;
    if (rem < 1e-6)
        return 0.0;
    return 15.0 - rem;
}

static void sleep_seconds(double sec)
{
    if (sec <= 0.0)
        return;

    struct timespec req = {0};
    req.tv_sec = (time_t)sec;
    req.tv_nsec = (long)((sec - (double)req.tv_sec) * 1000000000.0);

    while (nanosleep(&req, &req) != 0)
    {
        // Retry if interrupted.
        continue;
    }
}

static struct timespec mono_now(void)
{
    struct timespec ts = {0};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts;
}

static struct timespec mono_add_seconds(struct timespec t0, double sec)
{
    long add_sec = (long)sec;
    long add_nsec = (long)((sec - (double)add_sec) * 1000000000.0);
    t0.tv_sec += add_sec;
    t0.tv_nsec += add_nsec;
    while (t0.tv_nsec >= 1000000000L)
    {
        t0.tv_nsec -= 1000000000L;
        t0.tv_sec += 1;
    }
    return t0;
}

static void sleep_until_mono(const struct timespec* target)
{
    while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, target, NULL) == EINTR)
    {
        // Retry absolute sleep on interrupt.
        continue;
    }
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <wavfile> [base_freq_mhz] [slots]\n", argv[0]);
        fprintf(stderr, "  slots: 0 means run forever (default: 1)\n");
        return 2;
    }
    
    const char* wav_path = argv[1];
    float base_freq_mhz = 14.074f;
    if (argc >= 3) {
        base_freq_mhz = (float)atof(argv[2]);
    }
    int slots_to_run = 1;
    if (argc >= 4) {
        slots_to_run = atoi(argv[3]);
        if (slots_to_run < 0)
            slots_to_run = 1;
    }
    
    int fd = open(wav_path, O_RDONLY);
    if (fd < 0) {
        perror("open");
        return 1;
    }
    
    float* signal = NULL;
    int num_samples = 0;
    int num_channels = 0;
    int sample_rate = 0;
    
    if (load_wav(&signal, &num_samples, &num_channels, &sample_rate, wav_path, fd) != 0) {
        close(fd);
        fprintf(stderr, "Failed to load wav file: %s\n", wav_path);
        return 1;
    }
    close(fd);

    if (num_samples <= 0 || sample_rate <= 0) {
        free(signal);
        fprintf(stderr, "Invalid WAV content\n");
        return 1;
    }

    const int chunk_samples = 960;
    int16_t* chunk = (int16_t*)malloc((size_t)chunk_samples * sizeof(*chunk));
    if (chunk == NULL) {
        free(signal);
        fprintf(stderr, "Failed to allocate chunk buffer\n");
        return 1;
    }
    
    ft8_decode_context_t ctx = {0};
    ctx.is_ft8 = true;
    ctx.base_freq_mhz = base_freq_mhz;

    const int output_sample_rate = 8000;
    const int slot_samples = (int)lround(15.0 * (double)output_sample_rate);
    int source_pos = 0;
    int downsample_phase = 0;
    float current_sample = 0.0f;
    bool has_current_sample = false;
    int slot_index = 0;
    bool run_forever = (slots_to_run == 0);

    while (run_forever || slot_index < slots_to_run)
    {
        struct tm utc_now = {0};
        double frac_now = 0.0;
        get_utc_now(&utc_now, &frac_now);
        double wait_sec = seconds_to_next_ft8_slot(&utc_now, frac_now);
        if (wait_sec > 0.0)
        {
            OUT("slot %d: waiting %.3f s for next slot boundary (00/15/30/45)\n", slot_index + 1, wait_sec);
            sleep_seconds(wait_sec);
        }

        get_utc_now(&ctx.utc, &ctx.utc_frac_sec);
        OUT("slot %d: aligned UTC start %02d:%02d:%02d.%03d\n",
            slot_index + 1,
            ctx.utc.tm_hour,
            ctx.utc.tm_min,
            ctx.utc.tm_sec,
            (int)lround(ctx.utc_frac_sec * 1000.0));

        ft8_stream_decoder_t* stream = ft8_stream_open(output_sample_rate, &ctx);
        if (stream == nullptr)
        {
            free(chunk);
            free(signal);
            fprintf(stderr, "Failed to open stream decoder\n");
            return 1;
        }

        const struct timespec slot_start_mono = mono_now();
        int sent = 0;
        int blocks = 0;
        while (sent < slot_samples)
        {
            int n = slot_samples - sent;
            if (n > chunk_samples)
                n = chunk_samples;

            int filled = 0;
            while (filled < n)
            {
                if (!has_current_sample)
                {
                    current_sample = signal[source_pos];
                    ++source_pos;
                    if (source_pos >= num_samples)
                        source_pos = 0;
                    has_current_sample = true;
                }

                for (int dup = 0; dup < 2 && filled < n; ++dup)
                {
                    if (downsample_phase == 0)
                    {
                        float scaled = current_sample * 32767.0f;
                        if (scaled > 32767.0f)
                            scaled = 32767.0f;
                        if (scaled < -32768.0f)
                            scaled = -32768.0f;
                        chunk[filled++] = (int16_t)lroundf(scaled);
                    }
                    downsample_phase = (downsample_phase + 1) % 3;
                }

                has_current_sample = false;
            }

            int rc_append = ft8_stream_append_i16(stream, chunk, n);
            if (rc_append < 0)
            {
                ft8_stream_close(stream);
                free(chunk);
                free(signal);
                fprintf(stderr, "Stream append failed in slot %d at sample %d\n", slot_index + 1, sent);
                return 1;
            }

            blocks += rc_append;
            sent += n;

            struct timespec target = mono_add_seconds(slot_start_mono, (double)sent / (double)output_sample_rate);
            sleep_until_mono(&target);
        }

        int rc = ft8_stream_finalize(stream);
        ft8_stream_close(stream);
        OUT("slot %d: ingest complete (%d samples, %d-sample chunks, %d full blocks), finalize rc=%d\n",
            slot_index + 1,
            slot_samples,
            chunk_samples,
            blocks,
            rc);

        if (rc < 0)
        {
            free(chunk);
            free(signal);
            return 1;
        }

        ++slot_index;
    }

    free(chunk);
    free(signal);
    return 0;
}

#endif
