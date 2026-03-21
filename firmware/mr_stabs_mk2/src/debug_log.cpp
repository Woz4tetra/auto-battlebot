#include "debug_log.h"
#include <stdarg.h>
#include <stdio.h>

static char ring_buf[DEBUG_LOG_BUF_SIZE];
static size_t write_pos = 0;
static bool wrapped = false;

static char flat_buf[DEBUG_LOG_BUF_SIZE + 1];

void debug_log(const char *fmt, ...)
{
    char line[256];
    int prefix_len = snprintf(line, sizeof(line), "[%lu] ", (unsigned long)millis());

    va_list ap;
    va_start(ap, fmt);
    int msg_len = vsnprintf(line + prefix_len, sizeof(line) - prefix_len, fmt, ap);
    va_end(ap);

    int total = prefix_len + msg_len;
    if (total >= (int)sizeof(line))
        total = sizeof(line) - 1;

    if (total + 1 > (int)DEBUG_LOG_BUF_SIZE)
        return;

    for (int i = 0; i < total; i++)
    {
        ring_buf[write_pos] = line[i];
        write_pos++;
        if (write_pos >= DEBUG_LOG_BUF_SIZE)
        {
            write_pos = 0;
            wrapped = true;
        }
    }
    ring_buf[write_pos] = '\n';
    write_pos++;
    if (write_pos >= DEBUG_LOG_BUF_SIZE)
    {
        write_pos = 0;
        wrapped = true;
    }
}

const char *debug_log_get_all()
{
    if (!wrapped)
    {
        memcpy(flat_buf, ring_buf, write_pos);
        flat_buf[write_pos] = '\0';
    }
    else
    {
        size_t tail_len = DEBUG_LOG_BUF_SIZE - write_pos;
        memcpy(flat_buf, ring_buf + write_pos, tail_len);
        memcpy(flat_buf + tail_len, ring_buf, write_pos);
        flat_buf[DEBUG_LOG_BUF_SIZE] = '\0';
    }
    return flat_buf;
}

void debug_log_clear()
{
    write_pos = 0;
    wrapped = false;
}
