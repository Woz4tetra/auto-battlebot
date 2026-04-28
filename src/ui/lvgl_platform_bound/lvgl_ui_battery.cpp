#include "lvgl_platform_bound/lvgl_ui_battery.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <spdlog/spdlog.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <ctime>
#include <string>

#include "battery_soc_estimator.hpp"

namespace auto_battlebot::ui_internal {
namespace {

// INA219 register addresses from datasheet.
constexpr uint8_t INA219_REG_CONFIG = 0x00;
constexpr uint8_t INA219_REG_BUSVOLTAGE = 0x02;
constexpr uint8_t INA219_REG_POWER = 0x03;
constexpr uint8_t INA219_REG_CURRENT = 0x04;
constexpr uint8_t INA219_REG_CALIBRATION = 0x05;
// Calibration register value for the selected profile (16V bus range, up to ~5A expected current).
// Derived with Cal = trunc(0.04096 / (Current_LSB * Rshunt)).
constexpr uint16_t INA219_CALIBRATION_16V_5A = 26868;
// Config register bits matching the Waveshare reference setup (16V range, gain/ADC, continuous
// mode).
constexpr uint16_t INA219_CONFIG_16V_5A_CONTINUOUS = 0x0EEF;
// Datasheet constant used in the INA219 calibration equation.
constexpr double INA219_CALIBRATION_NUMERATOR = 0.04096;
// Shunt resistor value on the UPS board; used with calibration to convert raw current counts.
constexpr double INA219_SHUNT_RESISTANCE_OHM = 0.01;
// Current LSB derived from calibration: Current_LSB = 0.04096 / (Cal * Rshunt).
constexpr double INA219_CURRENT_LSB_A =
    INA219_CALIBRATION_NUMERATOR / (INA219_CALIBRATION_16V_5A * INA219_SHUNT_RESISTANCE_OHM);
// INA219 power register uses Power_LSB = 20 * Current_LSB.
constexpr double INA219_POWER_CURRENT_LSB_MULTIPLIER = 20.0;
constexpr double INA219_POWER_LSB_W = INA219_POWER_CURRENT_LSB_MULTIPLIER * INA219_CURRENT_LSB_A;
constexpr double kBatteryReadWarnMs = 30.0;

bool write_i2c_register_u16(int fd, uint8_t reg, uint16_t value) {
    uint8_t payload[3] = {reg, static_cast<uint8_t>((value >> 8) & 0xFF),
                          static_cast<uint8_t>(value & 0xFF)};
    return write(fd, payload, sizeof(payload)) == static_cast<ssize_t>(sizeof(payload));
}

bool read_i2c_register_u16(int fd, uint8_t reg, uint16_t &value) {
    uint8_t reg_addr = reg;
    if (write(fd, &reg_addr, sizeof(reg_addr)) != static_cast<ssize_t>(sizeof(reg_addr))) {
        return false;
    }
    uint8_t data[2] = {0, 0};
    if (read(fd, data, sizeof(data)) != static_cast<ssize_t>(sizeof(data))) {
        return false;
    }
    value = static_cast<uint16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]);
    return true;
}

bool read_waveshare_ups_sample(const BatteryOptions &options, BatterySample &sample_out) {
    const auto read_start = std::chrono::steady_clock::now();
    auto finish = [&](bool ok) {
        const double elapsed_ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - read_start)
                .count();
        if (elapsed_ms > kBatteryReadWarnMs) {
            spdlog::warn("Waveshare UPS read slow: elapsed_ms={:.2f} ok={}", elapsed_ms, ok);
        }
        return ok;
    };

    const int battery_i2c_bus = options.i2c_bus;
    const int battery_i2c_address = options.i2c_address;
    const std::string dev_path = "/dev/i2c-" + std::to_string(battery_i2c_bus);
    int fd = open(dev_path.c_str(), O_RDWR);
    if (fd < 0) return finish(false);
    if (ioctl(fd, I2C_SLAVE, battery_i2c_address) < 0) {
        close(fd);
        return finish(false);
    }

    bool ok = write_i2c_register_u16(fd, INA219_REG_CALIBRATION, INA219_CALIBRATION_16V_5A);
    if (ok) {
        ok = write_i2c_register_u16(fd, INA219_REG_CONFIG, INA219_CONFIG_16V_5A_CONTINUOUS);
    }
    uint16_t raw_bus_voltage = 0;
    uint16_t raw_current = 0;
    uint16_t raw_power = 0;
    if (ok) ok = read_i2c_register_u16(fd, INA219_REG_BUSVOLTAGE, raw_bus_voltage);
    if (ok) ok = read_i2c_register_u16(fd, INA219_REG_CURRENT, raw_current);
    if (ok) ok = read_i2c_register_u16(fd, INA219_REG_POWER, raw_power);
    close(fd);
    if (!ok) return finish(false);

    const int16_t current_signed = static_cast<int16_t>(raw_current);
    const int16_t power_signed = static_cast<int16_t>(raw_power);
    sample_out.timestamp = std::chrono::steady_clock::now();
    sample_out.raw_bus_voltage = raw_bus_voltage;
    sample_out.raw_current = current_signed;
    sample_out.raw_power = power_signed;
    sample_out.bus_voltage_v = static_cast<double>(raw_bus_voltage >> 3U) * 0.004;
    sample_out.current_a = static_cast<double>(current_signed) * INA219_CURRENT_LSB_A;
    sample_out.power_w = static_cast<double>(power_signed) * INA219_POWER_LSB_W;
    sample_out.valid = true;
    return finish(true);
}

class WaveshareUpsBatterySource : public IBatterySource {
   public:
    explicit WaveshareUpsBatterySource(const BatteryOptions &options)
        : options_(options), estimator_(options) {}

    BatteryReading read() override {
        BatterySample sample;
        const bool ok = read_waveshare_ups_sample(options_, sample);
        if (!ok) {
            sample.timestamp = std::chrono::steady_clock::now();
            sample.valid = false;
        }
        const BatteryReading reading = estimator_.update(sample);
        estimator_.log_diagnostics(sample, reading, options_, ok);
        return reading;
    }

   private:
    BatteryOptions options_;
    BatterySocEstimator estimator_;
};

class DummyBatterySource : public IBatterySource {
   public:
    explicit DummyBatterySource(double fixed_percent) : fixed_percent_(fixed_percent) {}

    BatteryReading read() override { return {.percent = fixed_percent_, .valid = true}; }

   private:
    double fixed_percent_ = 75.0;
};

void render_battery_canvas(UIWidgets &w, double percent, bool valid) {
    if (!w.battery_icon_canvas) return;
    lv_canvas_fill_bg(w.battery_icon_canvas, lv_color_hex(0x111111), LV_OPA_COVER);

    lv_layer_t layer;
    lv_canvas_init_layer(w.battery_icon_canvas, &layer);

    lv_draw_rect_dsc_t shell_dsc;
    lv_draw_rect_dsc_init(&shell_dsc);
    shell_dsc.radius = 4;
    shell_dsc.bg_color = lv_color_hex(0x1E1E1E);
    shell_dsc.bg_opa = LV_OPA_COVER;
    shell_dsc.border_width = 2;
    shell_dsc.border_color = lv_color_hex(0xE0E0E0);
    shell_dsc.border_opa = LV_OPA_COVER;
    const lv_area_t shell_area = {.x1 = BATTERY_ICON_SHELL_X,
                                  .y1 = BATTERY_ICON_SHELL_Y,
                                  .x2 = BATTERY_ICON_SHELL_X + BATTERY_ICON_SHELL_WIDTH - 1,
                                  .y2 = BATTERY_ICON_SHELL_Y + BATTERY_ICON_SHELL_HEIGHT - 1};
    lv_draw_rect(&layer, &shell_dsc, &shell_area);

    lv_color_t fill_color = lv_color_hex(0x00C853);
    if (!valid) {
        fill_color = lv_color_hex(0x616161);
    } else if (percent < 20.0) {
        fill_color = lv_color_hex(0xFF1744);
    } else if (percent < 50.0) {
        fill_color = lv_color_hex(0xFFC107);
    }
    const int fill_w =
        valid ? std::clamp(
                    static_cast<int>(std::lround((percent / 100.0) * BATTERY_ICON_FILL_MAX_WIDTH)),
                    0, BATTERY_ICON_FILL_MAX_WIDTH)
              : 0;
    if (fill_w > 0) {
        lv_draw_rect_dsc_t fill_dsc;
        lv_draw_rect_dsc_init(&fill_dsc);
        fill_dsc.radius = 2;
        fill_dsc.bg_color = fill_color;
        fill_dsc.bg_opa = LV_OPA_COVER;
        fill_dsc.border_width = 0;
        const lv_area_t fill_area = {
            .x1 = BATTERY_ICON_SHELL_X + 4,
            .y1 = BATTERY_ICON_SHELL_Y + (BATTERY_ICON_SHELL_HEIGHT - BATTERY_ICON_FILL_HEIGHT) / 2,
            .x2 = BATTERY_ICON_SHELL_X + 6 + fill_w - 1,
            .y2 = BATTERY_ICON_SHELL_Y +
                  (BATTERY_ICON_SHELL_HEIGHT - BATTERY_ICON_FILL_HEIGHT) / 2 +
                  BATTERY_ICON_FILL_HEIGHT - 1};
        lv_draw_rect(&layer, &fill_dsc, &fill_area);
    }

    lv_draw_rect_dsc_t cap_dsc;
    lv_draw_rect_dsc_init(&cap_dsc);
    cap_dsc.radius = 1;
    cap_dsc.bg_color = lv_color_hex(0xE0E0E0);
    cap_dsc.bg_opa = LV_OPA_COVER;
    cap_dsc.border_width = 0;
    const int cap_x = BATTERY_ICON_SHELL_X + BATTERY_ICON_SHELL_WIDTH + 2;
    const int cap_y = (BATTERY_ICON_CANVAS_HEIGHT - BATTERY_ICON_CAP_HEIGHT) / 2;
    const lv_area_t cap_area = {.x1 = cap_x,
                                .y1 = cap_y,
                                .x2 = cap_x + BATTERY_ICON_CAP_WIDTH - 1,
                                .y2 = cap_y + BATTERY_ICON_CAP_HEIGHT - 1};
    lv_draw_rect(&layer, &cap_dsc, &cap_area);
    lv_canvas_finish_layer(w.battery_icon_canvas, &layer);
}

void update_battery_widgets(UIWidgets &w, double percent, bool valid) {
    if (!w.battery_percent_label || !w.battery_icon_canvas) return;
    render_battery_canvas(w, percent, valid);

    lv_color_t color = lv_color_hex(0x00C853);
    if (!valid) {
        color = lv_color_hex(0x616161);
    } else if (percent < 20.0) {
        color = lv_color_hex(0xFF1744);
    } else if (percent < 50.0) {
        color = lv_color_hex(0xFFC107);
    }

    if (!valid) {
        lv_label_set_text(w.battery_percent_label, "--%");
        lv_obj_set_style_text_color(w.battery_percent_label, color, 0);
        return;
    }

    char percent_buf[8];
    snprintf(percent_buf, sizeof(percent_buf), "%d%%", static_cast<int>(std::lround(percent)));
    lv_label_set_text(w.battery_percent_label, percent_buf);
    lv_obj_set_style_text_color(w.battery_percent_label, lv_color_hex(0xE0E0E0), 0);
}

}  // namespace

std::string normalize_battery_source(std::string source) {
    std::transform(source.begin(), source.end(), source.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return source;
}

void build_top_bar(lv_obj_t *parent, UIWidgets &w) {
    lv_obj_t *bar = lv_obj_create(parent);
    w.top_bar = bar;
    lv_obj_set_width(bar, LV_PCT(100));
    lv_obj_set_height(bar, TOP_BAR_HEIGHT);
    lv_obj_set_style_pad_hor(bar, 12, 0);
    lv_obj_set_style_pad_ver(bar, 4, 0);
    lv_obj_set_style_pad_gap(bar, 8, 0);
    lv_obj_set_style_radius(bar, 0, 0);
    lv_obj_set_style_border_width(bar, 0, 0);
    lv_obj_set_style_bg_color(bar, lv_color_hex(0x111111), 0);
    lv_obj_clear_flag(bar, LV_OBJ_FLAG_SCROLLABLE);

    w.clock_label = lv_label_create(bar);
    lv_label_set_text(w.clock_label, "--:--:--");
    lv_obj_set_style_text_font(w.clock_label, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(w.clock_label, lv_color_hex(0xE0E0E0), 0);
    lv_obj_center(w.clock_label);

    lv_obj_t *battery_wrap = lv_obj_create(bar);
    lv_obj_set_height(battery_wrap, LV_SIZE_CONTENT);
    lv_obj_set_width(battery_wrap, LV_PCT(100));
    lv_obj_set_style_pad_all(battery_wrap, 0, 0);
    lv_obj_set_style_pad_gap(battery_wrap, 4, 0);
    lv_obj_set_style_radius(battery_wrap, 0, 0);
    lv_obj_set_style_border_width(battery_wrap, 0, 0);
    lv_obj_set_style_bg_opa(battery_wrap, LV_OPA_TRANSP, 0);
    lv_obj_set_flex_flow(battery_wrap, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(battery_wrap, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(battery_wrap, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_align(battery_wrap, LV_ALIGN_RIGHT_MID, -8, 0);

    w.battery_icon_canvas = lv_canvas_create(battery_wrap);
    lv_obj_set_size(w.battery_icon_canvas, BATTERY_ICON_CANVAS_WIDTH, BATTERY_ICON_CANVAS_HEIGHT);
    lv_obj_set_style_pad_all(w.battery_icon_canvas, 0, 0);
    lv_obj_set_style_bg_opa(w.battery_icon_canvas, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(w.battery_icon_canvas, 0, 0);
    lv_obj_clear_flag(w.battery_icon_canvas, LV_OBJ_FLAG_SCROLLABLE);
    lv_canvas_set_buffer(w.battery_icon_canvas, w.battery_icon_canvas_buf.data(),
                         BATTERY_ICON_CANVAS_WIDTH, BATTERY_ICON_CANVAS_HEIGHT,
                         LV_COLOR_FORMAT_RGB565);
    render_battery_canvas(w, w.dummy_battery_percent, true);

    w.battery_percent_label = lv_label_create(battery_wrap);
    lv_label_set_text(w.battery_percent_label, "--%");
    lv_obj_set_style_text_font(w.battery_percent_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(w.battery_percent_label, lv_color_hex(0xE0E0E0), 0);
}

void update_top_bar(UIWidgets &w, IBatterySource &battery_source) {
    const std::time_t now_secs = std::time(nullptr);
    if (w.clock_label && now_secs != w.last_clock_second) {
        std::tm local_tm{};
        localtime_r(&now_secs, &local_tm);
        char time_buf[16];
        if (std::strftime(time_buf, sizeof(time_buf), "%H:%M:%S", &local_tm) > 0) {
            lv_label_set_text(w.clock_label, time_buf);
        }
        w.last_clock_second = now_secs;
    }

    const auto now_steady = std::chrono::steady_clock::now();
    if (w.last_battery_update != std::chrono::steady_clock::time_point::min() &&
        std::chrono::duration_cast<std::chrono::milliseconds>(now_steady - w.last_battery_update)
                .count() < 1000) {
        return;
    }
    w.last_battery_update = now_steady;

    const BatteryReading reading = battery_source.read();
    update_battery_widgets(w, reading.percent, reading.valid);
}

std::unique_ptr<IBatterySource> make_battery_source(const std::string &normalized_source_name,
                                                    const BatteryOptions &options) {
    if (normalized_source_name == "waveshare ups" || normalized_source_name == "waveshareups") {
        return std::make_unique<WaveshareUpsBatterySource>(options);
    }
    return std::make_unique<DummyBatterySource>(75.0);
}

std::unique_ptr<IBatterySource> make_battery_source(const std::string &normalized_source_name) {
    return make_battery_source(normalized_source_name, BatteryOptions{});
}

}  // namespace auto_battlebot::ui_internal
