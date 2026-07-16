// auto-battlebot velocity jig firmware
//
// Feather RP2040 Adalogger. Logs encoder count and IMU (accel + gyro) together
// to the microSD card, one row per IMU sample. For an angular-only run, just
// unplug the encoder: its inputs are pulled up, so the count column stays
// constant instead of drifting.
//
// OLED menu: A = start recording, B = stop.
//
// Gap-free design:
//   - Every sample is captured in the IMU INT1 data-ready ISR, so SD write
//     latency never delays a capture. Captures land in a RAM ring buffer.
//   - The main loop drains the ring to SD in large blocks. A long SD stall just
//     lets the ring fill; the ISR keeps sampling. An overflow counter flags any
//     loss (it should stay 0).
//   - Files are preallocated contiguous to avoid FAT-growth latency, and the
//     card is on its own SPI peripheral (SPI0), separate from the IMU (SPI1).
//   - The screen is drawn once at record start and left untouched until stop.
//
// Wiring and pin map: docs/velocity_jig_wiring.md and include/config.h

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <SdFat.h>
#include <Adafruit_ISM330DHCX.h>
#include <Adafruit_SH110X.h>
#include "pico/time.h"

#include "config.h"

// --- Debug bring-up instrumentation (set -DVJIG_DEBUG=1 in platformio.ini) ---
#ifndef VJIG_DEBUG
#define VJIG_DEBUG 0
#endif
#if VJIG_DEBUG
#define DBG(x) Serial.print(x)
#define DBGLN(x) Serial.println(x)
#define DBGFLUSH() Serial.flush()
#else
#define DBG(x) ((void)0)
#define DBGLN(x) ((void)0)
#define DBGFLUSH() ((void)0)
#endif

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------

static Adafruit_ISM330DHCX g_imu;
static Adafruit_SH1107 g_display(64, 128, &Wire);
static SdFs g_sd;
static FsFile g_file;

// One row: IMU sample plus the encoder count captured at the same instant.
struct Sample {
    uint64_t t_us;  // capture time, microseconds since boot (time_us_64)
    int32_t count;  // quadrature count
    int16_t g[3];   // raw gyro x,y,z
    int16_t a[3];   // raw accel x,y,z
};

// SPSC ring buffer. Producer = IMU ISR, consumer = loop(). Single core, so the
// ISR atomically preempts the loop; volatile 32-bit indices are sufficient.
static Sample g_ring[RING_SLOTS];
static constexpr uint32_t RING_MASK = RING_SLOTS - 1;
static volatile uint32_t g_head = 0;      // producer writes
static volatile uint32_t g_tail = 0;      // consumer writes
static volatile uint32_t g_overflow = 0;  // dropped samples (ring full)
static volatile uint32_t g_produced = 0;  // samples captured this run

static bool g_recording = false;
static char g_curName[24] = {0};

// Encoder quadrature state, maintained continuously by edge interrupts.
static volatile int32_t g_encCount = 0;
static volatile uint8_t g_encState = 0;

// SD write staging buffer.
static char g_wbuf[WBUF_SIZE];
static uint32_t g_wlen = 0;

// ---------------------------------------------------------------------------
// Ring buffer
// ---------------------------------------------------------------------------

static inline void ringPush(const Sample& s) {
    uint32_t h = g_head;
    uint32_t n = (h + 1) & RING_MASK;
    if (n == g_tail) {  // full
        g_overflow++;
        return;
    }
    g_ring[h] = s;
    g_head = n;  // publish
    g_produced++;
}

// ---------------------------------------------------------------------------
// IMU raw access (fast burst read, bypasses the library for per-sample reads)
// ---------------------------------------------------------------------------

static void imuWriteReg(uint8_t reg, uint8_t val) {
    SPI1.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
    digitalWrite(PIN_IMU_CS, LOW);
    SPI1.transfer(reg & 0x7F);  // write: MSB clear
    SPI1.transfer(val);
    digitalWrite(PIN_IMU_CS, HIGH);
    SPI1.endTransaction();
}

#if VJIG_DEBUG
static uint8_t imuReadReg(uint8_t reg) {
    SPI1.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
    digitalWrite(PIN_IMU_CS, LOW);
    SPI1.transfer(0x80 | reg);  // read: MSB set
    uint8_t v = SPI1.transfer(0x00);
    digitalWrite(PIN_IMU_CS, HIGH);
    SPI1.endTransaction();
    return v;
}
#endif

// Burst-read gyro (0x22..0x27) then accel (0x28..0x2D), little-endian int16.
static inline void imuReadRaw(int16_t* g, int16_t* a) {
    uint8_t b[12];
    SPI1.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
    digitalWrite(PIN_IMU_CS, LOW);
    SPI1.transfer(0x80 | 0x22);  // read, auto-increment from OUTX_L_G
    for (int i = 0; i < 12; i++) b[i] = SPI1.transfer(0x00);
    digitalWrite(PIN_IMU_CS, HIGH);
    SPI1.endTransaction();
    g[0] = (int16_t)(b[0] | (b[1] << 8));
    g[1] = (int16_t)(b[2] | (b[3] << 8));
    g[2] = (int16_t)(b[4] | (b[5] << 8));
    a[0] = (int16_t)(b[6] | (b[7] << 8));
    a[1] = (int16_t)(b[8] | (b[9] << 8));
    a[2] = (int16_t)(b[10] | (b[11] << 8));
}

// ---------------------------------------------------------------------------
// Capture ISRs
// ---------------------------------------------------------------------------

// 4x quadrature decode table indexed by (prev_state << 2) | new_state.
static const int8_t QDEC[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

static void encISR() {
    uint8_t s = (uint8_t)((digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B));
    g_encCount += QDEC[((g_encState << 2) | s) & 0x0F];
    g_encState = s;
}

// Fires on IMU data-ready. Captures the IMU sample and the current encoder
// count together, then queues one row.
static void imuISR() {
    Sample s;
    s.t_us = time_us_64();
    s.count = g_encCount;
    imuReadRaw(s.g, s.a);
    ringPush(s);
}

// ---------------------------------------------------------------------------
// SD write staging (integer-only formatting -> light CPU load)
// ---------------------------------------------------------------------------

static inline void wflush() {
    if (g_wlen) {
        g_file.write((const uint8_t*)g_wbuf, g_wlen);
        g_wlen = 0;
    }
}

static inline void wputc(char c) {
    if (g_wlen >= WBUF_SIZE) wflush();
    g_wbuf[g_wlen++] = c;
}

static void wputU64(uint64_t v) {
    char tmp[20];
    int i = 0;
    if (v == 0) {
        wputc('0');
        return;
    }
    while (v) {
        tmp[i++] = (char)('0' + (v % 10));
        v /= 10;
    }
    while (i) wputc(tmp[--i]);
}

static void wputI32(int32_t v) {
    if (v < 0) {
        wputc('-');
        wputU64((uint64_t)(-(int64_t)v));
    } else {
        wputU64((uint64_t)v);
    }
}

static void wputStr(const char* s) {
    while (*s) wputc(*s++);
}

static inline void emitRow(const Sample& s) {
    wputU64(s.t_us);
    wputc(',');
    wputI32(s.count);
    for (int i = 0; i < 3; i++) {
        wputc(',');
        wputI32(s.g[i]);
    }
    for (int i = 0; i < 3; i++) {
        wputc(',');
        wputI32(s.a[i]);
    }
    wputc('\n');
}

// Drain up to DRAIN_CAP records to the staging buffer.
static void drainToSD() {
    uint32_t head = g_head;  // snapshot
    uint32_t processed = 0;
    while (g_tail != head && processed < DRAIN_CAP) {
        emitRow(g_ring[g_tail]);
        g_tail = (g_tail + 1) & RING_MASK;
        processed++;
    }
}

static void drainAll() {
    while (g_tail != g_head) drainToSD();
}

// ---------------------------------------------------------------------------
// OLED
// ---------------------------------------------------------------------------

static void drawMenu(bool withSummary) {
    g_display.clearDisplay();
    g_display.setTextSize(1);
    g_display.setTextColor(SH110X_WHITE);
    g_display.setCursor(0, 0);
    g_display.println("VELOCITY JIG");
    g_display.println();
    g_display.println("A: START REC");
    g_display.println("B: STOP");
    if (withSummary && g_curName[0]) {
        g_display.println();
        g_display.print("last ");
        g_display.println(g_curName);
        g_display.print("n=");
        g_display.println(g_produced);
        if (g_overflow) {
            g_display.print("DROPPED=");
            g_display.println(g_overflow);
        }
    }
    g_display.display();
}

static void drawRecording(const char* name) {
    g_display.clearDisplay();
    g_display.setTextSize(1);
    g_display.setTextColor(SH110X_WHITE);
    g_display.setCursor(0, 0);
    g_display.println("RECORDING");
    g_display.println("enc + imu");
    g_display.println(name);
    g_display.println();
    g_display.println("B: STOP");
    g_display.display();
}

// Fatal error: show a message and blink the LED forever.
static void fatal(const char* msg) {
    g_display.clearDisplay();
    g_display.setTextSize(1);
    g_display.setTextColor(SH110X_WHITE);
    g_display.setCursor(0, 0);
    g_display.println("ERROR");
    g_display.println(msg);
    g_display.display();
    Serial.print("FATAL: ");
    Serial.println(msg);
    pinMode(PIN_STATUS_LED, OUTPUT);
    uint32_t last = 0;
    for (;;) {
        digitalWrite(PIN_STATUS_LED, HIGH);
        delay(120);
        digitalWrite(PIN_STATUS_LED, LOW);
        delay(120);
        if (millis() - last >= 1000) {  // re-emit so a late-attached monitor sees it
            last = millis();
            Serial.print("FATAL: ");
            Serial.println(msg);
        }
    }
}

// ---------------------------------------------------------------------------
// File naming and header
// ---------------------------------------------------------------------------

static bool nextName(char* out, size_t n) {
    for (uint32_t i = 0; i < 100000; i++) {
        snprintf(out, n, "LOG-%lu.TXT", (unsigned long)i);
        if (!g_sd.exists(out)) return true;
    }
    return false;
}

static void writeHeader() {
    char line[128];
    wputStr("# auto-battlebot velocity jig\n");
    wputStr("# columns: t_us,count,gx,gy,gz,ax,ay,az  (raw int16 imu)\n");
    wputStr("# encoder: quadrature x4 (constant column if unplugged)\n");
    snprintf(line, sizeof(line),
             "# odr_hz=%lu gyro_dps_per_lsb=%s accel_g_per_lsb=%s\n",
             (unsigned long)IMU_ODR_HZ, GYRO_DPS_PER_LSB_STR, ACCEL_G_PER_LSB_STR);
    wputStr(line);
}

// ---------------------------------------------------------------------------
// Record start/stop
// ---------------------------------------------------------------------------

static void startRecording() {
    char name[24];
    if (!nextName(name, sizeof(name))) {
        drawMenu(false);
        return;
    }
    if (!g_file.open(name, O_RDWR | O_CREAT | O_TRUNC)) {
        Serial.println("open failed");
        drawMenu(false);
        return;
    }
    g_file.preAllocate(PREALLOC_BYTES);  // best effort; avoids FAT-growth stalls

    strncpy(g_curName, name, sizeof(g_curName) - 1);
    g_curName[sizeof(g_curName) - 1] = 0;

    // Reset buffers/counters before enabling the capture ISR.
    g_wlen = 0;
    g_head = g_tail = 0;
    g_overflow = 0;
    g_produced = 0;
    noInterrupts();
    g_encCount = 0;  // each file starts its count at 0
    interrupts();
    writeHeader();

    g_recording = true;
    attachInterrupt(PIN_IMU_INT1, imuISR, RISING);  // start capture
    drawRecording(name);                            // last screen update until STOP
    Serial.print("recording ");
    Serial.println(name);
}

static void stopRecording() {
    detachInterrupt(PIN_IMU_INT1);  // stop capture first
    g_recording = false;

    drainAll();
    wflush();
    g_file.sync();
    g_file.close();

    Serial.print("stopped, n=");
    Serial.print(g_produced);
    Serial.print(" dropped=");
    Serial.println(g_overflow);

    drawMenu(true);
}

// ---------------------------------------------------------------------------
// Buttons (debounced, active low)
// ---------------------------------------------------------------------------

struct Btn {
    uint8_t pin;
    bool lastRaw;
    bool state;
    uint32_t lastChange;
};

static Btn g_btnA{PIN_BTN_A, false, false, 0};
static Btn g_btnB{PIN_BTN_B, false, false, 0};

static bool justPressed(Btn& b) {
    bool raw = (digitalRead(b.pin) == LOW);
    uint32_t now = millis();
    if (raw != b.lastRaw) {
        b.lastRaw = raw;
        b.lastChange = now;
    }
    if ((now - b.lastChange) >= DEBOUNCE_MS && raw != b.state) {
        b.state = raw;
        if (raw) return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// Debug helpers (compiled out when VJIG_DEBUG == 0)
// ---------------------------------------------------------------------------

// Pump the USB stack and wait (bounded) for the serial monitor to attach. This
// is the key fix: delay() services USB, so the CDC port enumerates here even if
// a later init call hangs. Blinks the LED while waiting.
static void dbgWaitForHost(uint32_t timeout_ms) {
#if VJIG_DEBUG
    uint32_t t0 = millis();
    while (!Serial && (millis() - t0) < timeout_ms) {
        digitalWrite(PIN_STATUS_LED, (millis() / 100) & 1);
        delay(10);
    }
    digitalWrite(PIN_STATUS_LED, LOW);
#else
    (void)timeout_ms;
#endif
}

// Scan an I2C bus and print every responding address.
static void dbgI2CScan(TwoWire& w) {
#if VJIG_DEBUG
    Serial.print("[dbg] I2C scan:");
    Serial.flush();
    uint8_t found = 0;
    for (uint8_t a = 1; a < 127; a++) {
        w.beginTransmission(a);
        if (w.endTransmission() == 0) {
            Serial.print(" 0x");
            Serial.print(a, HEX);
            found++;
        }
    }
    Serial.println(found ? "" : " (none found)");
#else
    (void)w;
#endif
}

// ---------------------------------------------------------------------------
// Setup / loop
// ---------------------------------------------------------------------------

static void imuInit() {
    SPI1.setSCK(PIN_IMU_SCK);
    SPI1.setTX(PIN_IMU_MOSI);
    SPI1.setRX(PIN_IMU_MISO);
    SPI1.begin();
    pinMode(PIN_IMU_CS, OUTPUT);
    digitalWrite(PIN_IMU_CS, HIGH);
#if VJIG_DEBUG
    uint8_t who = imuReadReg(0x0F);  // WHO_AM_I, expect 0x6B
    Serial.print("[dbg] IMU WHO_AM_I=0x");
    Serial.print(who, HEX);
    Serial.println(who == 0x6B ? " (ISM330DHCX ok)" : " (want 0x6B; check SPI1 wiring/CS)");
    Serial.flush();
#endif
    DBG("[dbg] imu.begin_SPI... ");
    DBGFLUSH();
    if (!g_imu.begin_SPI(PIN_IMU_CS, &SPI1)) {
        fatal("IMU begin_SPI failed");
    }
    DBGLN("OK");
    g_imu.setAccelRange(IMU_ACCEL_RANGE);
    g_imu.setGyroRange(IMU_GYRO_RANGE);
    g_imu.setAccelDataRate(IMU_ODR);
    g_imu.setGyroDataRate(IMU_ODR);
    g_imu.configInt1(false, false, true);  // route accel data-ready to INT1
    imuWriteReg(0x12, 0x44);                // CTRL3_C: BDU=1, IF_INC=1
    pinMode(PIN_IMU_INT1, INPUT);
}

static void sdInit() {
    SPI.setSCK(PIN_SD_SCK);
    SPI.setTX(PIN_SD_MOSI);
    SPI.setRX(PIN_SD_MISO);
    DBG("[dbg] sd.begin (SPI0 GP18/19/20 CS23)... ");
    DBGFLUSH();
    // Dedicated SPI: the card owns SPI0, so SdFat keeps CS optimizations.
    if (!g_sd.begin(SdSpiConfig(PIN_SD_CS, DEDICATED_SPI, SD_SCK_MHZ(SD_SCK_MHZ_VAL), &SPI))) {
        fatal("SD init failed");
    }
    DBGLN("OK");
}

static void encoderInit() {
    // Pull-ups so a disconnected encoder reads a stable level (no phantom
    // counts). The count column then stays constant during angular-only runs.
    pinMode(PIN_ENC_A, INPUT_PULLUP);
    pinMode(PIN_ENC_B, INPUT_PULLUP);
    g_encState = (uint8_t)((digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B));
    attachInterrupt(PIN_ENC_A, encISR, CHANGE);
    attachInterrupt(PIN_ENC_B, encISR, CHANGE);
}

void setup() {
    Serial.begin(115200);
    pinMode(PIN_STATUS_LED, OUTPUT);
    digitalWrite(PIN_STATUS_LED, LOW);

    // Pump USB and wait (bounded) for the monitor before touching hardware, so
    // the serial port always enumerates even if a later init call hangs.
    dbgWaitForHost(8000);
    DBGLN("");
    DBGLN("=== velocity jig bring-up ===");

    pinMode(PIN_BTN_A, INPUT_PULLUP);
    pinMode(PIN_BTN_B, INPUT_PULLUP);
    pinMode(PIN_BTN_C, INPUT_PULLUP);

    // OLED on I2C1 (GP2/GP3). First a non-blocking bus-idle check: with internal
    // pull-ups both lines should read high when idle. A line stuck low means it
    // is held down (short, bad connection, or a device holding the bus), which is
    // exactly what hangs the first I2C transaction. Report instead of hanging.
    pinMode(PIN_I2C_SDA, INPUT_PULLUP);
    pinMode(PIN_I2C_SCL, INPUT_PULLUP);
    delay(5);
    int idleSda = digitalRead(PIN_I2C_SDA);
    int idleScl = digitalRead(PIN_I2C_SCL);
    DBG("[dbg] I2C idle SDA=");
    DBG(idleSda);
    DBG(" SCL=");
    DBGLN(idleScl);
    if (!idleSda || !idleScl) {
        for (uint32_t last = 0;;) {
            digitalWrite(PIN_STATUS_LED, (millis() / 300) & 1);
            if (millis() - last >= 1000) {
                last = millis();
                Serial.print("FATAL: I2C1 line stuck low (SDA=");
                Serial.print(idleSda);
                Serial.print(" SCL=");
                Serial.print(idleScl);
                Serial.println("). Check GP2/GP3 wiring, OLED seating, pull-ups.");
            }
            delay(10);
        }
    }

    DBGLN("[dbg] configuring Wire (GP2 SDA / GP3 SCL)");
    DBGFLUSH();
    Wire.setSDA(PIN_I2C_SDA);
    DBGLN("[dbg]   setSDA ok");
    DBGFLUSH();
    Wire.setSCL(PIN_I2C_SCL);
    DBGLN("[dbg]   setSCL ok");
    DBGFLUSH();
    Wire.setClock(100000);
    DBGLN("[dbg]   setClock ok");
    DBGFLUSH();
    Wire.begin();
    DBGLN("[dbg]   begin ok");
    DBGFLUSH();
    Wire.setTimeout(50, true);  // bound I2C so a bad bus reports instead of hanging
    DBGLN("[dbg]   setTimeout ok");
    DBGFLUSH();
    dbgI2CScan(Wire);

    DBG("[dbg] display.begin(0x3C)... ");
    DBGFLUSH();
    if (!g_display.begin(OLED_ADDR, true)) {
        DBGLN("FAIL");
        // The display is what fatal() draws on, so report over serial + LED only.
        uint32_t last = 0;
        for (;;) {
            digitalWrite(PIN_STATUS_LED, (millis() / 300) & 1);
            if (millis() - last >= 1000) {
                last = millis();
                Serial.println("FATAL: OLED not found on I2C1 @0x3C (check GP2/GP3 wiring)");
            }
            delay(10);
        }
    }
    DBGLN("OK");
    g_display.setRotation(1);
    g_display.clearDisplay();
    g_display.display();

    imuInit();
    sdInit();
    encoderInit();  // encoder tracks continuously, independent of recording

    DBGLN("[dbg] ready -> menu");
    drawMenu(false);
}

void loop() {
    if (g_recording) {
        drainToSD();
        if (justPressed(g_btnB)) stopRecording();
    } else {
        if (justPressed(g_btnA)) startRecording();
    }
}
