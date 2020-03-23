/*
 * Copyright (c) 2012 David Rodrigues
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

#ifndef CARLAVIZ_LOGGER_H_
#define CARLAVIZ_LOGGER_H_

#include <time.h>
#include <string.h>

// === auxiliar functions
static inline char *carlaviz_timenow();

#define CARLAVIZ_NO_LOGS         0x00
#define CARLAVIZ_ERROR_LEVEL     0x01
#define CARLAVIZ_WARNING_LEVEL   0x02
#define CARLAVIZ_INFO_LEVEL      0x03
#define CARLAVIZ_DEBUG_LEVEL     0x04

#ifndef CARLAVIZ_LOG_LEVEL
#define CARLAVIZ_LOG_LEVEL   CARLAVIZ_DEBUG_LEVEL
#endif


#define CARLAVIZ_PRINTFUNCTION(format, ...)      fprintf(stderr, format, __VA_ARGS__)


#define CARLAVIZ_LOG_FMT             "%s %-10s "
#define CARLAVIZ_LOG_ARGS(LOG_TAG)   carlaviz_timenow(), LOG_TAG

#define CARLAVIZ_NEWLINE     "\n"

#define CARLAVIZ_ERROR_TAG    "[ERROR]"
#define CARLAVIZ_WARNING_TAG  "[WARNING]"
#define CARLAVIZ_INFO_TAG     "[INFO]"
#define CARLAVIZ_DEBUG_TAG    "[DEBUG]"

#if CARLAVIZ_LOG_LEVEL >= CARLAVIZ_DEBUG_LEVEL
#define CARLAVIZ_LOG_DEBUG(message, args...)     CARLAVIZ_PRINTFUNCTION(CARLAVIZ_LOG_FMT message CARLAVIZ_NEWLINE, CARLAVIZ_LOG_ARGS(CARLAVIZ_DEBUG_TAG), ## args)
#else
#define CARLAVIZ_LOG_DEBUG(message, args...)
#endif

#if CARLAVIZ_LOG_LEVEL >= CARLAVIZ_INFO_LEVEL
#define CARLAVIZ_LOG_INFO(message, args...)      CARLAVIZ_PRINTFUNCTION(CARLAVIZ_LOG_FMT message CARLAVIZ_NEWLINE, CARLAVIZ_LOG_ARGS(CARLAVIZ_INFO_TAG), ## args)
#else
#define CARLAVIZ_LOG_INFO(message, args...)
#endif

#if CARLAVIZ_LOG_LEVEL >= CARLAVIZ_WARNING_LEVEL
#define CARLAVIZ_LOG_WARNING(message, args...)      CARLAVIZ_PRINTFUNCTION(CARLAVIZ_LOG_FMT message CARLAVIZ_NEWLINE, CARLAVIZ_LOG_ARGS(CARLAVIZ_WARNING_TAG), ## args)
#else
#define CARLAVIZ_LOG_WARNING(message, args...)
#endif

#if CARLAVIZ_LOG_LEVEL >= CARLAVIZ_ERROR_LEVEL
#define CARLAVIZ_LOG_ERROR(message, args...)     CARLAVIZ_PRINTFUNCTION(CARLAVIZ_LOG_FMT message CARLAVIZ_NEWLINE, CARLAVIZ_LOG_ARGS(CARLAVIZ_ERROR_TAG), ## args)
#else
#define CARLAVIZ_LOG_ERROR(message, args...)
#endif

#if CARLAVIZ_LOG_LEVEL >= CARLAVIZ_NO_LOGS
#define CARLAVIZ_LOG_IF_ERROR(condition, message, args...) if (condition) CARLAVIZ_PRINTFUNCTION(CARLAVIZ_LOG_FMT message CARLAVIZ_NEWLINE, CARLAVIZ_LOG_ARGS(CARLAVIZ_ERROR_TAG), ## args)
#else
#define CARLAVIZ_LOG_IF_ERROR(condition, message, args...)
#endif

static inline char *carlaviz_timenow() {
    static char buffer[64];
    time_t rawtime;
    struct tm *timeinfo;
    
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    
    strftime(buffer, 64, "%Y-%m-%d %H:%M:%S", timeinfo);
    
    return buffer;
}

#endif