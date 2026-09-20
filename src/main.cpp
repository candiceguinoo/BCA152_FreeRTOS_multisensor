/* =====================================================================
 *  BCA152 - FreeRTOS Room Multisensor  (single-file TEST version)
 *
 *  Sections:  1 Settings   2 Shared types   3 DHT22   4 LDR
 *             5 SensorTask 6 OLED driver    7 DisplayTask
 *             8 InputTask  9 app_main
 * ===================================================================== */

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"

/* ============================ 1. SETTINGS ============================ */
// Pins: must match diagram.json
static const gpio_num_t    DHT22_PIN    = GPIO_NUM_4;
static const adc_channel_t LDR_CHANNEL  = ADC_CHANNEL_6;   // ADC1 ch6 = GPIO34
static const gpio_num_t    OLED_SDA_PIN = GPIO_NUM_21;
static const gpio_num_t    OLED_SCL_PIN = GPIO_NUM_22;
static const gpio_num_t    ENC_CLK_PIN  = GPIO_NUM_32;
static const gpio_num_t    ENC_DT_PIN   = GPIO_NUM_33;

#define OLED_I2C_ADDR     0x3C
#define SENSOR_PERIOD_MS  2000

// Task priorities (bigger = more urgent)
#define PRIO_INPUT    3   // must react quickly to encoder clicks
#define PRIO_SENSOR   2   // runs every 2 s, small delays are fine
#define PRIO_DISPLAY  1   // screen refresh is the least urgent

/* ========================= 2. SHARED TYPES =========================== */
struct SensorData {
    float temperature;     // Celsius (NAN until first good reading)
    float humidity;        // percent (NAN until first good reading)
    int   lightLevel;      // relative brightness 0-100 %
    bool  motionDetected;  // PIR is added in Part IX
};

enum class DisplayMode { TEMPERATURE, HUMIDITY, LIGHT, MOTION };

// Pure logic: no hardware in here, so it can be unit tested later.
static DisplayMode nextDisplayMode(DisplayMode current)
{
    switch (current) {
        case DisplayMode::TEMPERATURE: return DisplayMode::HUMIDITY;
        case DisplayMode::HUMIDITY:    return DisplayMode::LIGHT;
        case DisplayMode::LIGHT:       return DisplayMode::MOTION;
        case DisplayMode::MOTION:      return DisplayMode::TEMPERATURE;  // wraparound
    }
    return DisplayMode::TEMPERATURE;
}

static DisplayMode previousDisplayMode(DisplayMode current)
{
    switch (current) {
        case DisplayMode::TEMPERATURE: return DisplayMode::MOTION;       // wraparound
        case DisplayMode::HUMIDITY:    return DisplayMode::TEMPERATURE;
        case DisplayMode::LIGHT:       return DisplayMode::HUMIDITY;
        case DisplayMode::MOTION:      return DisplayMode::LIGHT;
    }
    return DisplayMode::TEMPERATURE;
}

// Queues shared between tasks
static QueueHandle_t g_sensorQueue      = nullptr;  // SensorTask -> DisplayTask
static QueueHandle_t g_displayModeQueue = nullptr;  // InputTask  -> DisplayTask (length 1)
static QueueHandle_t g_encoderQueue     = nullptr;  // encoder ISR -> InputTask

/* ============================ 3. DHT22 =============================== */
static const char *TAG_DHT = "dht22";

// Timing is measured in microseconds, so interrupts are switched off briefly.
static portMUX_TYPE s_dhtLock = portMUX_INITIALIZER_UNLOCKED;

// Wait until the pin reaches 'level'; report how long we waited.
static bool awaitLevel(int level, uint32_t timeoutUs, uint32_t *waitedUs)
{
    int64_t start = esp_timer_get_time();
    while (gpio_get_level(DHT22_PIN) != level) {
        if (esp_timer_get_time() - start > (int64_t)timeoutUs) {
            return false;
        }
    }
    if (waitedUs != nullptr) {
        *waitedUs = (uint32_t)(esp_timer_get_time() - start);
    }
    return true;
}

static void dht22Init(void)
{
    gpio_config_t cfg = {};
    cfg.pin_bit_mask = 1ULL << DHT22_PIN;
    cfg.mode = GPIO_MODE_INPUT_OUTPUT_OD;
    cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&cfg));
    gpio_set_level(DHT22_PIN, 1);
}

static bool dht22Read(float *temperature, float *humidity)
{
    uint8_t data[5] = {0, 0, 0, 0, 0};
    bool ok = true;

    portENTER_CRITICAL(&s_dhtLock);

    // 1) Wake the sensor: pull low 2 ms, then release.
    gpio_set_level(DHT22_PIN, 0);
    esp_rom_delay_us(2000);
    gpio_set_level(DHT22_PIN, 1);

    // 2) Sensor answers: low ~80 us, high ~80 us, then data starts.
    ok = awaitLevel(1, 60, nullptr) &&
         awaitLevel(0, 100, nullptr) &&
         awaitLevel(1, 120, nullptr) &&
         awaitLevel(0, 120, nullptr);

    // 3) 40 bits: high pulse longer than the low pulse -> 1, else 0.
    for (int i = 0; ok && i < 40; i++) {
        uint32_t lowUs = 0;
        uint32_t highUs = 0;
        ok = awaitLevel(1, 100, &lowUs) && awaitLevel(0, 120, &highUs);
        if (ok) {
            data[i / 8] = (uint8_t)(data[i / 8] << 1);
            if (highUs > lowUs) {
                data[i / 8] |= 1;
            }
        }
    }

    portEXIT_CRITICAL(&s_dhtLock);

    if (!ok) {
        ESP_LOGW(TAG_DHT, "no answer from DHT22 (timeout)");
        return false;
    }

    // 4) Checksum: 5th byte = low 8 bits of the sum of the first four.
    uint8_t sum = (uint8_t)(data[0] + data[1] + data[2] + data[3]);
    if (sum != data[4]) {
        ESP_LOGW(TAG_DHT, "checksum error");
        return false;
    }

    // 5) Convert (values are sent multiplied by 10).
    float h = (float)((data[0] << 8) | data[1]) / 10.0f;
    float t = (float)(((data[2] & 0x7F) << 8) | data[3]) / 10.0f;
    if (data[2] & 0x80) {
        t = -t;                       // top bit = negative temperature
    }

    // 6) Sanity check: reject impossible values.
    if (h < 0.0f || h > 100.0f || t < -40.0f || t > 80.0f) {
        ESP_LOGW(TAG_DHT, "value out of range (T=%.1f H=%.1f)", t, h);
        return false;
    }

    *temperature = t;
    *humidity = h;
    return true;
}

/* ============================== 4. LDR =============================== */
static const char *TAG_LDR = "ldr";
static adc_oneshot_unit_handle_t s_adc = nullptr;

static void ldrInit(void)
{
    adc_oneshot_unit_init_cfg_t unitCfg = {};
    unitCfg.unit_id = ADC_UNIT_1;
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unitCfg, &s_adc));

    adc_oneshot_chan_cfg_t chanCfg = {};
    chanCfg.atten = ADC_ATTEN_DB_12;
    chanCfg.bitwidth = ADC_BITWIDTH_DEFAULT;
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, LDR_CHANNEL, &chanCfg));
}

// Relative brightness 0 (dark) .. 100 (bright). -1 on error.
static int ldrReadPercent(void)
{
    int raw = 0;
    esp_err_t err = adc_oneshot_read(s_adc, LDR_CHANNEL, &raw);
    if (err != ESP_OK) {
        ESP_LOGW(TAG_LDR, "ADC read failed: %s", esp_err_to_name(err));
        return -1;
    }
    int percent = 100 - (raw * 100) / 4095;   // Wokwi module: dark = high voltage
    if (percent < 0)   percent = 0;
    if (percent > 100) percent = 100;
    return percent;
}

/* ========================== 5. SENSOR TASK =========================== */
static const char *TAG_SENSOR = "SensorTask";

static void sensorTask(void *pvParameters)
{
    (void)pvParameters;
    SensorData data = {NAN, NAN, 0, false};

    vTaskDelay(pdMS_TO_TICKS(1000));                  // DHT22 wake-up time
    TickType_t lastWakeTime = xTaskGetTickCount();

    for (;;) {
        float t = 0.0f;
        float h = 0.0f;
        if (dht22Read(&t, &h)) {
            data.temperature = t;
            data.humidity = h;
            ESP_LOGI(TAG_SENSOR, "Temperature: %.2f C  Humidity: %.2f %%", t, h);
        } else {
            ESP_LOGW(TAG_SENSOR, "DHT22 read failed - keeping last values");
        }

        int light = ldrReadPercent();
        if (light >= 0) {
            data.lightLevel = light;
        }
        ESP_LOGI(TAG_SENSOR, "Light: %d %%", data.lightLevel);

        if (xQueueSend(g_sensorQueue, &data, 0) != pdTRUE) {   // 0 = never wait
            ESP_LOGW(TAG_SENSOR, "sensor queue full - reading dropped");
        }

        // Sleep (Blocked) until exactly SENSOR_PERIOD_MS after the last wake-up.
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(SENSOR_PERIOD_MS));
    }
}

/* ========================== 6. OLED DRIVER =========================== */
static const char *TAG_OLED = "oled";
static const int OLED_WIDTH  = 128;
static const int OLED_HEIGHT = 64;
static const int OLED_PAGES  = OLED_HEIGHT / 8;

static i2c_master_bus_handle_t s_i2cBus = nullptr;
static i2c_master_dev_handle_t s_oled   = nullptr;
static uint8_t s_fb[OLED_WIDTH * OLED_PAGES];       // frame buffer (1 bit per pixel)

// 5x7 font, one byte per column (bit 0 = top row). Uppercase, digits, few symbols.
struct Glyph { char ch; uint8_t col[5]; };
static const Glyph FONT[] = {
    {' ', {0x00, 0x00, 0x00, 0x00, 0x00}}, {'.', {0x00, 0x60, 0x60, 0x00, 0x00}},
    {'%', {0x23, 0x13, 0x08, 0x64, 0x62}}, {':', {0x00, 0x36, 0x36, 0x00, 0x00}},
    {'-', {0x08, 0x08, 0x08, 0x08, 0x08}}, {'/', {0x20, 0x10, 0x08, 0x04, 0x02}},
    {'0', {0x3E, 0x51, 0x49, 0x45, 0x3E}}, {'1', {0x00, 0x42, 0x7F, 0x40, 0x00}},
    {'2', {0x42, 0x61, 0x51, 0x49, 0x46}}, {'3', {0x21, 0x41, 0x45, 0x4B, 0x31}},
    {'4', {0x18, 0x14, 0x12, 0x7F, 0x10}}, {'5', {0x27, 0x45, 0x45, 0x45, 0x39}},
    {'6', {0x3C, 0x4A, 0x49, 0x49, 0x30}}, {'7', {0x01, 0x71, 0x09, 0x05, 0x03}},
    {'8', {0x36, 0x49, 0x49, 0x49, 0x36}}, {'9', {0x06, 0x49, 0x49, 0x29, 0x1E}},
    {'A', {0x7E, 0x11, 0x11, 0x11, 0x7E}}, {'B', {0x7F, 0x49, 0x49, 0x49, 0x36}},
    {'C', {0x3E, 0x41, 0x41, 0x41, 0x22}}, {'D', {0x7F, 0x41, 0x41, 0x22, 0x1C}},
    {'E', {0x7F, 0x49, 0x49, 0x49, 0x41}}, {'F', {0x7F, 0x09, 0x09, 0x09, 0x01}},
    {'G', {0x3E, 0x41, 0x49, 0x49, 0x7A}}, {'H', {0x7F, 0x08, 0x08, 0x08, 0x7F}},
    {'I', {0x00, 0x41, 0x7F, 0x41, 0x00}}, {'J', {0x20, 0x40, 0x41, 0x3F, 0x01}},
    {'K', {0x7F, 0x08, 0x14, 0x22, 0x41}}, {'L', {0x7F, 0x40, 0x40, 0x40, 0x40}},
    {'M', {0x7F, 0x02, 0x0C, 0x02, 0x7F}}, {'N', {0x7F, 0x04, 0x08, 0x10, 0x7F}},
    {'O', {0x3E, 0x41, 0x41, 0x41, 0x3E}}, {'P', {0x7F, 0x09, 0x09, 0x09, 0x06}},
    {'Q', {0x3E, 0x41, 0x51, 0x21, 0x5E}}, {'R', {0x7F, 0x09, 0x19, 0x29, 0x46}},
    {'S', {0x46, 0x49, 0x49, 0x49, 0x31}}, {'T', {0x01, 0x01, 0x7F, 0x01, 0x01}},
    {'U', {0x3F, 0x40, 0x40, 0x40, 0x3F}}, {'V', {0x1F, 0x20, 0x40, 0x20, 0x1F}},
    {'W', {0x3F, 0x40, 0x38, 0x40, 0x3F}}, {'X', {0x63, 0x14, 0x08, 0x14, 0x63}},
    {'Y', {0x07, 0x08, 0x70, 0x08, 0x07}}, {'Z', {0x61, 0x51, 0x49, 0x45, 0x43}},
};

static const uint8_t *findGlyph(char c)
{
    if (c >= 'a' && c <= 'z') {
        c = (char)(c - 32);                       // lowercase -> uppercase
    }
    for (size_t i = 0; i < sizeof(FONT) / sizeof(FONT[0]); i++) {
        if (FONT[i].ch == c) {
            return FONT[i].col;
        }
    }
    return FONT[0].col;                           // unknown character -> blank
}

// Send command bytes (control byte 0x00 = "these are commands").
static void oledCommands(const uint8_t *cmds, size_t n)
{
    uint8_t buf[40];
    if (n > sizeof(buf) - 1) {
        return;
    }
    buf[0] = 0x00;
    memcpy(&buf[1], cmds, n);
    esp_err_t err = i2c_master_transmit(s_oled, buf, n + 1, 100);
    if (err != ESP_OK) {
        ESP_LOGW(TAG_OLED, "I2C command failed: %s", esp_err_to_name(err));
    }
}

static void oledInit(void)
{
    i2c_master_bus_config_t busCfg = {};
    busCfg.i2c_port = I2C_NUM_0;
    busCfg.sda_io_num = OLED_SDA_PIN;
    busCfg.scl_io_num = OLED_SCL_PIN;
    busCfg.clk_source = I2C_CLK_SRC_DEFAULT;
    busCfg.glitch_ignore_cnt = 7;
    busCfg.flags.enable_internal_pullup = true;
    ESP_ERROR_CHECK(i2c_new_master_bus(&busCfg, &s_i2cBus));

    i2c_device_config_t devCfg = {};
    devCfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    devCfg.device_address = OLED_I2C_ADDR;
    devCfg.scl_speed_hz = 400000;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_i2cBus, &devCfg, &s_oled));

    static const uint8_t initSeq[] = {
        0xAE,             // display off
        0xD5, 0x80,       // clock divide
        0xA8, 0x3F,       // multiplex: 64 rows
        0xD3, 0x00,       // display offset
        0x40,             // start line
        0x8D, 0x14,       // charge pump on
        0x20, 0x02,       // page addressing mode
        0xA1,             // flip columns
        0xC8,             // flip rows
        0xDA, 0x12,       // COM pins
        0x81, 0xCF,       // contrast
        0xD9, 0xF1,       // pre-charge
        0xDB, 0x40,       // VCOMH
        0xA4,             // show RAM content
        0xA6,             // normal (not inverted)
        0xAF              // display on
    };
    oledCommands(initSeq, sizeof(initSeq));
}

static void oledClear(void)
{
    memset(s_fb, 0, sizeof(s_fb));
}

static void oledPixel(int x, int y)
{
    if (x < 0 || x >= OLED_WIDTH || y < 0 || y >= OLED_HEIGHT) {
        return;
    }
    s_fb[(y / 8) * OLED_WIDTH + x] |= (uint8_t)(1 << (y % 8));
}

static void oledDrawChar(int x, int y, char c, int scale)
{
    const uint8_t *cols = findGlyph(c);
    for (int col = 0; col < 5; col++) {
        for (int row = 0; row < 7; row++) {
            if (cols[col] & (1 << row)) {
                for (int dx = 0; dx < scale; dx++) {
                    for (int dy = 0; dy < scale; dy++) {
                        oledPixel(x + col * scale + dx, y + row * scale + dy);
                    }
                }
            }
        }
    }
}

static void oledDrawText(int x, int y, const char *text, int scale)
{
    for (; *text != '\0'; text++) {
        oledDrawChar(x, y, *text, scale);
        x += 6 * scale;                           // 5 pixel glyph + 1 pixel gap
    }
}

static void oledDrawHLine(int y)
{
    for (int x = 0; x < OLED_WIDTH; x++) {
        oledPixel(x, y);
    }
}

// Copy the frame buffer to the screen, one page (8 pixel rows) at a time.
static void oledFlush(void)
{
    uint8_t buf[1 + OLED_WIDTH];
    for (int page = 0; page < OLED_PAGES; page++) {
        const uint8_t setPos[] = {(uint8_t)(0xB0 | page), 0x00, 0x10};
        oledCommands(setPos, sizeof(setPos));

        buf[0] = 0x40;                            // control byte 0x40 = "pixel data"
        memcpy(&buf[1], &s_fb[page * OLED_WIDTH], OLED_WIDTH);
        esp_err_t err = i2c_master_transmit(s_oled, buf, sizeof(buf), 100);
        if (err != ESP_OK) {
            ESP_LOGW(TAG_OLED, "I2C data failed: %s", esp_err_to_name(err));
            return;
        }
    }
}

/* ========================= 7. DISPLAY TASK =========================== */
static void renderScreen(DisplayMode mode, const SensorData &d)
{
    const char *label = "";
    char value[48] = "";      // roomy on purpose: avoids -Wformat-truncation errors
    char page[16];            // roomy on purpose: avoids -Wformat-truncation errors

    switch (mode) {
        case DisplayMode::TEMPERATURE:
            label = "TEMPERATURE";
            if (std::isnan(d.temperature)) snprintf(value, sizeof(value), "-- C");
            else snprintf(value, sizeof(value), "%.1f C", d.temperature);
            break;
        case DisplayMode::HUMIDITY:
            label = "HUMIDITY";
            if (std::isnan(d.humidity)) snprintf(value, sizeof(value), "-- %%");
            else snprintf(value, sizeof(value), "%.1f %%", d.humidity);
            break;
        case DisplayMode::LIGHT:
            label = "LIGHT";
            snprintf(value, sizeof(value), "%d %%", d.lightLevel);
            break;
        case DisplayMode::MOTION:
            label = "MOTION";
            snprintf(value, sizeof(value), "%s", d.motionDetected ? "YES" : "NONE");
            break;
    }
    snprintf(page, sizeof(page), "%d/4", (int)mode + 1);

    oledClear();
    oledDrawText(0, 0, "ROOM MONITOR", 1);
    oledDrawText(110, 0, page, 1);
    oledDrawHLine(10);
    oledDrawText(0, 16, label, 1);
    oledDrawText(0, 30, value, 3);
    oledDrawText(0, 56, "TURN KNOB TO CHANGE", 1);
    oledFlush();
}

// DisplayTask is the ONLY task that touches the OLED.
static void displayTask(void *pvParameters)
{
    (void)pvParameters;
    SensorData data = {NAN, NAN, 0, false};
    DisplayMode mode = DisplayMode::TEMPERATURE;
    bool redraw = true;

    oledInit();

    for (;;) {
        if (redraw) {
            renderScreen(mode, data);
            redraw = false;
        }

        // BLOCKED here for up to 50 ms waiting for a page change...
        DisplayMode newMode;
        if (xQueueReceive(g_displayModeQueue, &newMode, pdMS_TO_TICKS(50)) == pdTRUE) {
            mode = newMode;
            redraw = true;
        }
        // ...then take any new sensor readings (0 = don't wait).
        SensorData newData;
        while (xQueueReceive(g_sensorQueue, &newData, 0) == pdTRUE) {
            data = newData;
            redraw = true;
        }
    }
}

/* =========================== 8. INPUT TASK =========================== */
enum class EncoderEvent : uint8_t { CLOCKWISE, COUNTER_CLOCKWISE };

// Runs by itself when CLK falls. Keep it SHORT: no printf, no delays.
static void IRAM_ATTR encoderIsr(void *arg)
{
    (void)arg;
    EncoderEvent ev = (gpio_get_level(ENC_DT_PIN) == 1)
                          ? EncoderEvent::CLOCKWISE
                          : EncoderEvent::COUNTER_CLOCKWISE;
    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(g_encoderQueue, &ev, &woken);
    portYIELD_FROM_ISR(woken);
}

static void encoderInit(void)
{
    gpio_config_t cfg = {};
    cfg.pin_bit_mask = (1ULL << ENC_CLK_PIN) | (1ULL << ENC_DT_PIN);
    cfg.mode = GPIO_MODE_INPUT;
    cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&cfg));

    ESP_ERROR_CHECK(gpio_set_intr_type(ENC_CLK_PIN, GPIO_INTR_NEGEDGE));

    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {   // "already installed" is fine
        ESP_ERROR_CHECK(err);
    }
    ESP_ERROR_CHECK(gpio_isr_handler_add(ENC_CLK_PIN, encoderIsr, nullptr));
}

static void inputTask(void *pvParameters)
{
    (void)pvParameters;
    DisplayMode mode = DisplayMode::TEMPERATURE;
    EncoderEvent ev;

    for (;;) {
        // BLOCKED here (0% CPU) until the ISR sends an event.
        if (xQueueReceive(g_encoderQueue, &ev, portMAX_DELAY) == pdTRUE) {
            if (ev == EncoderEvent::CLOCKWISE) {
                mode = nextDisplayMode(mode);
                printf("Clockwise -> DisplayMode: %d\n", (int)mode);
            } else {
                mode = previousDisplayMode(mode);
                printf("Counterclockwise -> DisplayMode: %d\n", (int)mode);
            }
            xQueueOverwrite(g_displayModeQueue, &mode);   // keep only the newest page
        }
    }
}

/* ============================ 9. APP_MAIN ============================ */
extern "C" void app_main(void)
{
    printf("BCA152 FreeRTOS Multisensor\n");
    printf("System starting...\n");

    // 1) Create the FreeRTOS objects first.
    g_sensorQueue      = xQueueCreate(5, sizeof(SensorData));
    g_displayModeQueue = xQueueCreate(1, sizeof(DisplayMode));
    g_encoderQueue     = xQueueCreate(16, sizeof(EncoderEvent));
    if (g_sensorQueue == nullptr || g_displayModeQueue == nullptr || g_encoderQueue == nullptr) {
        printf("ERROR: could not create queues\n");
        return;
    }

    // 2) Initialize hardware (the OLED is initialized by DisplayTask itself).
    dht22Init();
    ldrInit();
    encoderInit();

    // 3) Create the tasks: function, name, stack bytes, parameter, priority, handle.
    xTaskCreate(sensorTask,  "SensorTask",  4096, nullptr, PRIO_SENSOR,  nullptr);
    xTaskCreate(displayTask, "DisplayTask", 6144, nullptr, PRIO_DISPLAY, nullptr);  // bigger stack: text + I2C
    xTaskCreate(inputTask,   "InputTask",   4096, nullptr, PRIO_INPUT,   nullptr);

    // app_main may return; the tasks keep running.
}