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

#include "ft8/decoder_api.h"
#include "common/debug.h"
#include "secrets.h"

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

// WAV loader for LittleFS
// Returns heap-allocated float samples (caller must free), or nullptr on error.
static float* load_wav_littlefs(const char* path, int* out_num_samples, int* out_num_channels, int* out_sample_rate)
{
    File f = LittleFS.open(path, "r");
    if (!f) {
        Serial.printf("[ft8] Cannot open %s\n", path);
        return nullptr;
    }
    
    // Read and parse WAV header (RIFF container)
    uint8_t riff_header[12];
    if (f.read(riff_header, 12) != 12 || memcmp(riff_header, "RIFF", 4) != 0 || memcmp(riff_header + 8, "WAVE", 4) != 0) {
        Serial.println("[ft8] Not a valid WAVE file");
        f.close();
        return nullptr;
    }
    
    // Read chunks until we find "fmt " and "data"
    uint8_t chunk_header[8];
    uint32_t chunk_size;
    uint16_t audio_format, num_channels, bits_per_sample;
    uint32_t sample_rate;
    bool found_fmt = false, found_data = false;
    
    while (f.read(chunk_header, 8) == 8) {
        chunk_size = (chunk_header[7] << 24) | (chunk_header[6] << 16) | (chunk_header[5] << 8) | chunk_header[4];
        
        if (memcmp(chunk_header, "fmt ", 4) == 0) {
            uint8_t fmt_data[16];
            if (f.read(fmt_data, 16) != 16) break;
            audio_format = (fmt_data[1] << 8) | fmt_data[0];
            num_channels = (fmt_data[3] << 8) | fmt_data[2];
            sample_rate = (fmt_data[7] << 24) | (fmt_data[6] << 16) | (fmt_data[5] << 8) | fmt_data[4];
            bits_per_sample = (fmt_data[15] << 8) | fmt_data[14];
            found_fmt = true;
            if (chunk_size > 16) f.seek(f.position() + chunk_size - 16);
        } else if (memcmp(chunk_header, "data", 4) == 0) {
            found_data = true;
            break;
        } else {
            f.seek(f.position() + chunk_size);
        }
    }
    
    if (!found_fmt || !found_data) {
        Serial.println("[ft8] Missing fmt or data chunk");
        f.close();
        return nullptr;
    }
    
    if (audio_format != 1 || bits_per_sample != 16) {
        Serial.printf("[ft8] Unsupported format: %u-bit, format %u\n", bits_per_sample, audio_format);
        f.close();
        return nullptr;
    }
    
    uint32_t num_samples = chunk_size / (num_channels * (bits_per_sample / 8));
    float* samples = (float*)malloc(num_samples * sizeof(float));
    if (!samples) {
        Serial.println("[ft8] Failed to allocate sample buffer");
        f.close();
        return nullptr;
    }
    
    int16_t raw_sample;
    for (uint32_t i = 0; i < num_samples; i++) {
        if (f.read((uint8_t*)&raw_sample, 2) != 2) {
            Serial.println("[ft8] Unexpected EOF reading samples");
            free(samples);
            f.close();
            return nullptr;
        }
        samples[i] = (float)raw_sample / 32768.0f;
        
        // Skip remaining channels if multichannel
        for (int ch = 1; ch < num_channels; ch++) {
            if (f.read((uint8_t*)&raw_sample, 2) != 2) {
                Serial.println("[ft8] Unexpected EOF reading channels");
                free(samples);
                f.close();
                return nullptr;
            }
        }
    }
    
    f.close();
    *out_num_samples = num_samples;
    *out_num_channels = num_channels;
    *out_sample_rate = sample_rate;
    return samples;
}

static void decode_file(const char* path, float base_freq_mhz, bool is_ft8)
{
    Serial.printf("\n[ft8] Decoding %s (base %.3f MHz, %s)\n", path, base_freq_mhz, is_ft8 ? "FT8" : "FT4");
    int num_samples = 0, num_channels = 0, sample_rate = 0;
    float* signal = load_wav_littlefs(path, &num_samples, &num_channels, &sample_rate);
    if (!signal) return;
    
    Serial.printf("[ft8] Loaded: %d samples @ %d Hz, %d ch, free heap: %lu bytes\n", 
                  num_samples, sample_rate, num_channels, (unsigned long)ESP.getFreeHeap());
    
    ft8_decode_context_t ctx = {};
    ctx.is_ft8 = is_ft8;
    ctx.base_freq_mhz = base_freq_mhz;

    const int chunk_samples = 960;
    const int slot_samples = (int)(15.0f * sample_rate + 0.5f);
    const int symbol_samples = (int)(0.160f * sample_rate + 0.5f);
    const int checkpoint_samples = is_ft8 ? (79 * symbol_samples) : 0;
    const int slots_to_run = 1; // Set to 0 to run forever on this file.
    float* chunk = alloc_sample_buffer((size_t)chunk_samples);
    if (!chunk) {
        Serial.println("[ft8] Failed to allocate chunk buffer");
        free(signal);
        return;
    }

    int source_pos = 0;
    int slot_index = 0;
    while (slots_to_run == 0 || slot_index < slots_to_run)
    {
        struct tm utc_now = {0};
        double frac_now = 0.0;
        get_utc_now(&utc_now, &frac_now);
        double wait_sec = seconds_to_next_ft8_slot(&utc_now, frac_now);
        if (wait_sec > 0.0) {
            Serial.printf("[ft8] Slot %d waiting %.3f s for boundary 00/15/30/45\n", slot_index + 1, wait_sec);
            sleep_seconds(wait_sec);
        }

        get_utc_now(&ctx.utc, &ctx.utc_frac_sec);
        Serial.printf("[ft8] Slot %d aligned UTC start %02d:%02d:%02d.%03d\n",
                      slot_index + 1,
                      ctx.utc.tm_hour,
                      ctx.utc.tm_min,
                      ctx.utc.tm_sec,
                      (int)(ctx.utc_frac_sec * 1000.0 + 0.5));

        ft8_stream_decoder_t* stream = ft8_stream_open(sample_rate, &ctx);
        if (stream == nullptr) {
            Serial.println("[ft8] Failed to open stream decoder");
            free(chunk);
            free(signal);
            return;
        }

        int sent = 0;
        int blocks = 0;
        unsigned long t0 = millis();
        int64_t next_chunk_deadline_us = esp_timer_get_time();
        int chunk_index = 0;
        bool checkpoint_mark_logged = false;
        if (checkpoint_samples > 0)
        {
            Serial.printf("[ft8] Slot %d checkpoint target: %d samples (%d symbols)\n",
                          slot_index + 1,
                          checkpoint_samples,
                          79);
        }
        while (sent < slot_samples) {
            int n = slot_samples - sent;
            if (n > chunk_samples) {
                n = chunk_samples;
            }

            for (int i = 0; i < n; ++i) {
                chunk[i] = signal[source_pos++];
                if (source_pos >= num_samples) {
                    source_pos = 0;
                }
            }

            Serial.printf("[ft8] Slot %d  %d..%d (%.2f%%)\n", slot_index + 1, sent, sent + n, (float)(sent + n) * 100.0f / (float)slot_samples);
            int rc_append = ft8_stream_append_float(stream, chunk, n);
            
            if (rc_append < 0) {
                Serial.printf("[ft8] Stream append failed in slot %d at sample %d\n", slot_index + 1, sent);
                ft8_stream_close(stream);
                free(chunk);
                free(signal);
                return;
            }

            blocks += rc_append;
            int sent_before = sent;
            sent += n;
            ++chunk_index;

            if (!checkpoint_mark_logged && checkpoint_samples > 0 && sent_before < checkpoint_samples && sent >= checkpoint_samples)
            {
                Serial.printf("[ft8] Slot %d crossed checkpoint sample count: %d/%d\n",
                              slot_index + 1,
                              sent,
                              checkpoint_samples);
                checkpoint_mark_logged = true;
            }

            next_chunk_deadline_us += (int64_t)n * 1000000LL / (int64_t)sample_rate;
            sleep_until_mono_us(next_chunk_deadline_us);
        }

        int rc = ft8_stream_finalize(stream);
        ft8_stream_close(stream);
        unsigned long elapsed = millis() - t0;

        Serial.printf("[ft8] Slot %d ingest complete: %d samples in %d-sample chunks, %d full blocks\n",
                      slot_index + 1, slot_samples, chunk_samples, blocks);
        Serial.printf("[ft8] Slot %d done in %lu ms, rc=%d, free heap: %lu bytes\n",
                      slot_index + 1, elapsed, rc, (unsigned long)ESP.getFreeHeap());

        ++slot_index;
    }

    free(chunk);
    free(signal);
}

static void decode_task(void* /*arg*/)
{
    if (!LittleFS.begin(true)) {
        Serial.println("[ft8] LittleFS mount failed - reflash filesystem");
        vTaskDelete(nullptr);
        return;
    }
    
    Serial.println("[ft8] LittleFS mounted");
    File root = LittleFS.open("/");
    File entry;
    while ((entry = root.openNextFile())) {
        String name = String("/") + entry.name();
        entry.close();
        if (name.endsWith(".wav") || name.endsWith(".WAV")) {
            decode_file(name.c_str(), 14.074f, true);
        }
    }
    
    Serial.println("\n[ft8] All files decoded.");
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
    
    const uint32_t decode_stack_bytes = 32768;
    BaseType_t rc = xTaskCreatePinnedToCore(
        decode_task,
        "ft8_decode",
        decode_stack_bytes,
        nullptr,
        1,
        nullptr,
        APP_CPU_NUM
    );
    if (rc != pdPASS) {
        Serial.printf("[ft8] Failed to create decode task (rc=%ld)\n", (long)rc);
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
    float* chunk = alloc_sample_buffer((size_t)chunk_samples);
    if (chunk == NULL) {
        free(signal);
        fprintf(stderr, "Failed to allocate chunk buffer\n");
        return 1;
    }
    
    ft8_decode_context_t ctx = {0};
    ctx.is_ft8 = true;
    ctx.base_freq_mhz = base_freq_mhz;

    const int slot_samples = (int)lround(15.0 * (double)sample_rate);
    int source_pos = 0;
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

        ft8_stream_decoder_t* stream = ft8_stream_open(sample_rate, &ctx);
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

            for (int i = 0; i < n; ++i)
            {
                chunk[i] = signal[source_pos];
                ++source_pos;
                if (source_pos >= num_samples)
                    source_pos = 0;
            }

            int rc_append = ft8_stream_append_float(stream, chunk, n);
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

            struct timespec target = mono_add_seconds(slot_start_mono, (double)sent / (double)sample_rate);
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
