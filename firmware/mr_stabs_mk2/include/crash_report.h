#pragma once

// Why the robot last rebooted, and the core dump a crash left in flash.
//
// A panic or a task watchdog timeout writes an ELF core dump to the coredump partition
// (partitions.csv), then reboots. read() picks it up on the next boot and keeps the summary for
// the diagnostics page; the full dump, with every task's stack, stays in flash until erased.
// Decode it over USB with scripts/coredump.

#include <Arduino.h>

namespace crash_report {

struct report_t {
    uint8_t reset_reason;  // esp_reset_reason_t
    bool has_dump;
    bool dump_valid;  // checksum matched; false means a dump was cut short
    uint32_t dump_size;
    char task[16];  // task running on the crashing core
    uint32_t pc;
    uint32_t exc_cause;
    uint32_t exc_vaddr;
    uint32_t backtrace[16];
    uint8_t depth;
    bool backtrace_corrupted;
    char elf_sha256[17];  // first 16 hex characters of the crashing build's ELF SHA-256
};

/** Reads the reset reason and any core dump. Call once in setup(). */
void read(report_t &out);

/** Prints the report to Serial, one line per field. */
void print(const report_t &report);

/** Name for an esp_reset_reason_t, e.g. "TASK_WDT". */
const char *reset_reason_name(uint8_t reason);

/** Erases the dump in flash. Returns false if there was none or the erase failed. */
bool erase_dump();

}  // namespace crash_report
