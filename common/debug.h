#pragma once

#include <stdio.h>

#if defined(NATIVE_BUILD)
#include <time.h>
#endif

#define LOG_DEBUG   0
#define LOG_INFO    1
#define LOG_WARN    2
#define LOG_ERROR   3
#define LOG_FATAL   4


#if defined(NATIVE_BUILD)
static inline void log_native_timestamp_prefix(FILE* stream)
{
	struct timespec ts = {0};
	if (clock_gettime(CLOCK_REALTIME, &ts) != 0) {
		return;
	}

	time_t sec = ts.tv_sec;
	struct tm tm_now = {0};
	if (localtime_r(&sec, &tm_now) == NULL) {
		return;
	}

	int msec = (int)(ts.tv_nsec / 1000000L);

	fprintf(stream,
			"%04d/%02d/%02d %02d:%02d:%02d.%03d ",
			tm_now.tm_year + 1900,
			tm_now.tm_mon + 1,
			tm_now.tm_mday,
			tm_now.tm_hour,
			tm_now.tm_min,
			tm_now.tm_sec,
			msec);
}

#define LOG(level, ...)                                                      \
	do {                                                                     \
		if ((level) >= LOG_LEVEL) {                                          \
			log_native_timestamp_prefix(stderr);                             \
			fprintf(stderr, __VA_ARGS__);                                    \
			fflush(stderr);                                                   \
		}                                                                    \
	} while (0)

#define OUT(...)                                                              \
	do {                                                                     \
		log_native_timestamp_prefix(stdout);                                 \
		fprintf(stdout, __VA_ARGS__);                                        \
		fflush(stdout);                                                       \
	} while (0)
#else
#define LOG(level, ...)     if ((level) >= LOG_LEVEL) fprintf(stderr, __VA_ARGS__)
#define OUT(...)            fprintf(stdout, __VA_ARGS__)
#endif
