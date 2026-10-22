#include <crash_report.h>
#include <esp_core_dump.h>
#include <esp_system.h>

namespace crash_report {

const char *reset_reason_name(uint8_t reason) {
    switch (reason) {
        case ESP_RST_POWERON:
            return "POWERON";
        case ESP_RST_EXT:
            return "EXTERNAL";
        case ESP_RST_SW:
            return "SOFTWARE";
        case ESP_RST_PANIC:
            return "PANIC";
        case ESP_RST_INT_WDT:
            return "INT_WDT";
        case ESP_RST_TASK_WDT:
            return "TASK_WDT";
        case ESP_RST_WDT:
            return "OTHER_WDT";
        case ESP_RST_DEEPSLEEP:
            return "DEEPSLEEP";
        case ESP_RST_BROWNOUT:
            return "BROWNOUT";
        case ESP_RST_SDIO:
            return "SDIO";
        default:
            return "UNKNOWN";
    }
}

void read(report_t &out) {
    out = {};
    out.reset_reason = static_cast<uint8_t>(esp_reset_reason());

    size_t addr = 0;
    size_t size = 0;
    if (esp_core_dump_image_get(&addr, &size) != ESP_OK || size == 0) return;
    out.has_dump = true;
    out.dump_size = size;
    out.dump_valid = esp_core_dump_image_check() == ESP_OK;
    if (!out.dump_valid) return;

    // The summary is large for the stack; it is read once, at boot.
    static esp_core_dump_summary_t summary;
    if (esp_core_dump_get_summary(&summary) != ESP_OK) return;
    strncpy(out.task, summary.exc_task, sizeof(out.task) - 1);
    out.pc = summary.exc_pc;
    out.exc_cause = summary.ex_info.exc_cause;
    out.exc_vaddr = summary.ex_info.exc_vaddr;
    const uint32_t depth = min<uint32_t>(summary.exc_bt_info.depth, 16);
    for (uint32_t i = 0; i < depth; i++) out.backtrace[i] = summary.exc_bt_info.bt[i];
    out.depth = static_cast<uint8_t>(depth);
    out.backtrace_corrupted = summary.exc_bt_info.corrupted;
    memcpy(out.elf_sha256, summary.app_elf_sha256, sizeof(out.elf_sha256) - 1);
}

void print(const report_t &report) {
    Serial.printf("reset reason: %s\n", reset_reason_name(report.reset_reason));
    if (!report.has_dump) {
        Serial.println("core dump: none");
        return;
    }
    if (!report.dump_valid) {
        Serial.printf("core dump: %u bytes, checksum FAILED\n", report.dump_size);
        return;
    }
    Serial.printf("core dump: %u bytes, task %s, pc 0x%08x, cause %u, vaddr 0x%08x, elf %s\n",
                  report.dump_size, report.task, report.pc, report.exc_cause, report.exc_vaddr,
                  report.elf_sha256);
    Serial.print("backtrace:");
    for (uint8_t i = 0; i < report.depth; i++) Serial.printf(" 0x%08x", report.backtrace[i]);
    Serial.println(report.backtrace_corrupted ? " (corrupted)" : "");
}

bool erase_dump() { return esp_core_dump_image_erase() == ESP_OK; }

}  // namespace crash_report
