#include "ft8_consumer_module.h"

#include <math.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

#include <Arduino.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "ft8_decode/decoder_api.h"

typedef struct {
    ft8_stream_decoder_t* stream;
    int slot_index;
    int slot_samples;
    int ingested_samples;
    int blocks;
} finalize_job_t;

static QueueHandle_t s_sample_queue = nullptr;
static QueueHandle_t s_finalize_queue = nullptr;

static int s_sample_rate = 0;
static float s_base_freq_mhz = 14.074f;
static int s_append_batch_size = 256;

static int s_consumer_task_stack = 32768;
static int s_finalize_task_stack = 32768;
static UBaseType_t s_consumer_task_priority = 2;
static UBaseType_t s_finalize_task_priority = 1;
static BaseType_t s_consumer_task_core = APP_CPU_NUM;
static BaseType_t s_finalize_task_core = APP_CPU_NUM;

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

static void sleep_seconds(double sec)
{
    if (sec <= 0.0)
        return;

    uint32_t ms = (uint32_t)(sec * 1000.0);
    if (ms > 0)
        delay(ms);

    double rem = sec - ((double)ms / 1000.0);
    if (rem > 0.0)
    {
        uint32_t us = (uint32_t)(rem * 1000000.0);
        if (us > 0)
            delayMicroseconds(us);
    }
}

static void decoder_consumer_task(void* /*arg*/)
{
    const bool is_ft8 = true;
    const int slot_samples = (int)(15.0f * s_sample_rate + 0.5f);
    const int symbol_samples = (int)(0.160f * s_sample_rate + 0.5f);
    const int checkpoint_samples = is_ft8 ? (79 * symbol_samples) : 0;

    int slot_index = 0;
    struct timeval tv_now = {0};
    gettimeofday(&tv_now, nullptr);
    double now_epoch0 = (double)tv_now.tv_sec + (double)tv_now.tv_usec / 1000000.0;
    double next_slot_epoch = floor(now_epoch0 / 15.0) * 15.0;
    if (now_epoch0 - next_slot_epoch > 1e-6)
        next_slot_epoch += 15.0;

    int16_t* append_batch = (int16_t*)malloc((size_t)s_append_batch_size * sizeof(*append_batch));
    if (append_batch == nullptr)
    {
        Serial.println("[ft8] consumer: append batch allocation failed");
        vTaskDelete(nullptr);
        return;
    }

    for (;;)
    {
        gettimeofday(&tv_now, nullptr);
        double now_epoch = (double)tv_now.tv_sec + (double)tv_now.tv_usec / 1000000.0;

        // Resync in O(1): wall clock can jump on first NTP sync.
        if (now_epoch >= (next_slot_epoch + 15.0))
        {
            next_slot_epoch = floor(now_epoch / 15.0) * 15.0;
            if (now_epoch - next_slot_epoch > 1e-6)
                next_slot_epoch += 15.0;
        }

        double wait_sec = next_slot_epoch - now_epoch;
        while (wait_sec > 0.0)
        {
            TickType_t wait_ticks = pdMS_TO_TICKS((wait_sec > 0.010) ? 10 : (uint32_t)(wait_sec * 1000.0));
            if (wait_ticks < 1)
                wait_ticks = 1;
            vTaskDelay(wait_ticks);
            gettimeofday(&tv_now, nullptr);
            now_epoch = (double)tv_now.tv_sec + (double)tv_now.tv_usec / 1000000.0;
            wait_sec = next_slot_epoch - now_epoch;
        }

        gettimeofday(&tv_now, nullptr);
        now_epoch = (double)tv_now.tv_sec + (double)tv_now.tv_usec / 1000000.0;

        time_t slot_start_sec = (time_t)next_slot_epoch;
        struct tm utc_start = {0};
        gmtime_r(&slot_start_sec, &utc_start);
        double utc_frac_start = next_slot_epoch - (double)slot_start_sec;

        double slot_end_epoch = next_slot_epoch + 15.0;
        double remain_slot_sec = slot_end_epoch - now_epoch;
        if (remain_slot_sec < 0.0)
            remain_slot_sec = 0.0;

        int64_t slot_end_us = esp_timer_get_time() + (int64_t)(remain_slot_sec * 1000000.0 + 0.5);

        char slot_time_buf[32] = {0};
        format_utc_hhmmss_mmm(slot_time_buf, sizeof(slot_time_buf), tv_now);
        Serial.printf("[ft8] Slot start #%d @ %s (%d Hz)\n", slot_index + 1, slot_time_buf, s_sample_rate);

        ft8_decode_context_t ctx = {};
        ctx.is_ft8 = is_ft8;
        ctx.base_freq_mhz = s_base_freq_mhz;
        ctx.utc = utc_start;
        ctx.utc_frac_sec = utc_frac_start;

        ft8_stream_decoder_t* stream = ft8_stream_open(s_sample_rate, &ctx);
        if (stream == nullptr)
        {
            Serial.println("[ft8] consumer: failed to open stream decoder");
            ++slot_index;
            continue;
        }

        int ingested_samples = 0;
        int blocks = 0;
        int samples_since_yield = 0;
        bool checkpoint_logged = false;
        bool ingest_failed = false;

        for (;;)
        {
            
            int64_t now_us = esp_timer_get_time();
            if (now_us >= slot_end_us)
                break;

            int64_t remain_us = slot_end_us - now_us;
            TickType_t wait_ticks = pdMS_TO_TICKS((remain_us > 5000LL) ? 5 : (remain_us / 1000LL));
            if (wait_ticks < 1)
                wait_ticks = 1;

            int16_t first_sample = 0;
            if (xQueueReceive(s_sample_queue, &first_sample, wait_ticks) != pdTRUE)
                continue;

            // Serial.printf("[ft8] consumer: got %d samples from queue, remain slot time %.3f s\n", 1, (double)(slot_end_us - esp_timer_get_time()) / 1000000.0); 

            int batch_count = 0;
            append_batch[batch_count++] = first_sample;

            while (batch_count < s_append_batch_size)
            {
                int16_t sample = 0;
                if (xQueueReceive(s_sample_queue, &sample, 0) != pdTRUE)
                    break;
                append_batch[batch_count++] = sample;
            }

            int rc_append = ft8_stream_append_i16(stream, append_batch, batch_count);
            if (rc_append < 0)
            {
                Serial.printf("[ft8] consumer: stream append failed at sample %d\n", ingested_samples + batch_count - 1);
                ingest_failed = true;
                break;
            }

            ingested_samples += batch_count;
            blocks += rc_append;
            samples_since_yield += batch_count;

            // Allow IDLE0 to run while queue is continuously full during active slots.
            if (samples_since_yield >= 512)
            {
                vTaskDelay(1);
                samples_since_yield = 0;
            }

            if (!checkpoint_logged && checkpoint_samples > 0 && ingested_samples >= checkpoint_samples)
            {
                Serial.printf("[ft8] Slot #%d reached checkpoint (%d samples / 79 symbols)\n", slot_index + 1, ingested_samples);
                checkpoint_logged = true;
            }
        }

        if (ingest_failed || ingested_samples == 0)
        {
            ft8_stream_close(stream);
        }
        else
        {
            finalize_job_t fin = {};
            fin.stream = stream;
            fin.slot_index = slot_index + 1;
            fin.slot_samples = slot_samples;
            fin.ingested_samples = ingested_samples;
            fin.blocks = blocks;

            // Keep consumer slot timing stable: never block here waiting for finalize queue space.
            if (xQueueSend(s_finalize_queue, &fin, 0) != pdTRUE)
            {
                Serial.printf("[ft8] finalize queue full, dropping slot-%d finalize job\n", fin.slot_index);
                ft8_stream_close(stream);
            }
        }

        next_slot_epoch += 15.0;
        ++slot_index;
    }
}

static void finalize_worker_task(void* /*arg*/)
{
    for (;;)
    {
        finalize_job_t fin = {};
        if (xQueueReceive(s_finalize_queue, &fin, portMAX_DELAY) != pdTRUE)
            continue;

        int rc_final = ft8_stream_finalize(fin.stream);
        ft8_stream_close(fin.stream);

        Serial.printf("[ft8] Slot done slot-%d: %d samples ingested, %d full blocks, finalize rc=%d\n",
                      fin.slot_index,
                      fin.ingested_samples,
                      fin.blocks,
                      rc_final);
    }
}

bool ft8_consumer_module_init(const ft8_consumer_module_config_t* cfg)
{
    if (cfg == nullptr)
        return false;
    if (cfg->sample_rate <= 0 || cfg->sample_queue_depth <= 0 || cfg->finalize_queue_depth <= 0 || cfg->append_batch_size <= 0)
        return false;

    s_sample_rate = cfg->sample_rate;
    s_base_freq_mhz = cfg->base_freq_mhz;
    s_append_batch_size = cfg->append_batch_size;
    s_consumer_task_stack = cfg->consumer_task_stack;
    s_finalize_task_stack = cfg->finalize_task_stack;
    s_consumer_task_priority = cfg->consumer_task_priority;
    s_finalize_task_priority = cfg->finalize_task_priority;
    s_consumer_task_core = cfg->consumer_task_core;
    s_finalize_task_core = cfg->finalize_task_core;

    s_sample_queue = xQueueCreate((UBaseType_t)cfg->sample_queue_depth, sizeof(int16_t));
    s_finalize_queue = xQueueCreate((UBaseType_t)cfg->finalize_queue_depth, sizeof(finalize_job_t));

    if (s_sample_queue == nullptr || s_finalize_queue == nullptr)
    {
        Serial.println("[ft8] consumer: queue creation failed");
        return false;
    }

    return true;
}

bool ft8_consumer_module_start(void)
{
    if (s_sample_queue == nullptr || s_finalize_queue == nullptr)
        return false;

    BaseType_t rc_consumer = xTaskCreatePinnedToCore(
        decoder_consumer_task,
        "ft8_consumer",
        (uint32_t)s_consumer_task_stack,
        nullptr,
        s_consumer_task_priority,
        nullptr,
        s_consumer_task_core);

    BaseType_t rc_finalize = xTaskCreatePinnedToCore(
        finalize_worker_task,
        "ft8_finalize",
        (uint32_t)s_finalize_task_stack,
        nullptr,
        s_finalize_task_priority,
        nullptr,
        s_finalize_task_core);

    if (rc_consumer != pdPASS || rc_finalize != pdPASS)
    {
        Serial.printf("[ft8] consumer: task creation failed (consumer=%ld finalize=%ld)\n",
                      (long)rc_consumer,
                      (long)rc_finalize);
        return false;
    }

    return true;
}

int ft8_consumer_module_enqueue_i16(const int16_t* samples, int count, TickType_t timeout_ticks)
{
    if (s_sample_queue == nullptr || samples == nullptr || count < 0)
        return -1;

    for (int i = 0; i < count; ++i)
    {
        if (xQueueSend(s_sample_queue, &samples[i], timeout_ticks) != pdTRUE)
            return i;
    }

    return count;
}
