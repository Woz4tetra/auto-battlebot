#pragma once

#include <Arduino.h>

namespace crash_report {

struct report_t {
    uint8_t reset_reason;  // esp_reset_reason_t
    bool has_dump;
    bool dump_checksum_ok;
    uint32_t dump_size;
    char crashing_task_name[16];
    uint32_t pc;
    uint32_t exc_cause;
    uint32_t exc_vaddr;
    uint32_t backtrace[16];
    uint8_t depth;
    bool backtrace_corrupted;
    char elf_sha256_prefix[17];
};

void read(report_t &out);

void print(const report_t &report);

const char *reset_reason_name(uint8_t reason);

bool erase_dump();

}  // namespace crash_report
