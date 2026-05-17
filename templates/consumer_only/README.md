# Consumer-Only FT8 Skeleton

This folder contains a minimal consumer-side module extracted from `src/main.cpp`.
It is intentionally outside `src/` so it will not compile into this repository build.

Files:
- `ft8_consumer_module.h`
- `ft8_consumer_module.cpp`

## What this template does

- Keeps NTP-aligned slot starts at `00/15/30/45`.
- Opens one stream decoder per slot with `ft8_stream_open`.
- Appends `int16_t` samples via `ft8_stream_append_i16`.
- Finalizes in a dedicated worker task via `ft8_stream_finalize`.

## What you need to connect in your final project

1. Initialize Wi-Fi + NTP in your app startup.
2. Call `ft8_consumer_module_init(...)` and `ft8_consumer_module_start()` once.
3. Feed real audio from your I2S/ADC producer using:
   - `ft8_consumer_module_enqueue_i16(samples, count, timeout_ticks)`
4. Use `sample_rate = 8000` and `base_freq_mhz = 14.074f` (or your target band).

## Suggested config values

- `sample_queue_depth`: `32768`
- `finalize_queue_depth`: `4`
- `append_batch_size`: `256`
- `consumer_task_stack`: `32768`
- `finalize_task_stack`: `32768`
- `consumer_task_priority`: `2`
- `finalize_task_priority`: `1`
- core pinning: same core for both tasks, or match your system scheduling plan.

## Notes

- This is a skeleton, not a drop-in library package.
- Keep `ft8/`, `fft/`, and `common/` in your final project, plus `src/decode_ft8.c`.
