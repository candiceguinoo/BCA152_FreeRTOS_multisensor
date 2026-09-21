#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_attr.h"
#include "esp_err.h"

#include "alarm.h"

// ====================================================
// PIN DEFINITIONS
// ====================================================

#define DHT_PIN GPIO_NUM_4

#define LDR_ADC_CHANNEL ADC_CHANNEL_6   // GPIO34

#define OLED_SDA GPIO_NUM_21
#define OLED_SCL GPIO_NUM_22
#define OLED_ADDR 0x3C
#define I2C_PORT I2C_NUM_0

// Rotary Encoder
#define ENCODER_CLK GPIO_NUM_32
#define ENCODER_DT  GPIO_NUM_33
#define ENCODER_SW  GPIO_NUM_25

// PIR motion sensor
#define PIR_PIN GPIO_NUM_27

// ====================================================
// DHT22 SETTINGS
// ====================================================

#define DHT_TIMEOUT_US 150
#define DHT_BIT_TIMEOUT_US 120
#define DHT_ONE_THRESHOLD_US 50

// ====================================================
// TIMING SETTINGS
// ====================================================

#define INACTIVITY_TIMEOUT_US 15000000LL

// ====================================================
// ENCODER SETTINGS
// ====================================================

constexpr bool ENCODER_REVERSE = false;

// ====================================================
// ENUMERATIONS
// ====================================================

enum class DisplayMode
{
    TEMPERATURE,
    HUMIDITY,
    LIGHT,
    MOTION
};

enum class SystemState
{
    ACTIVE,
    INACTIVE
};

// ====================================================
// EVENT GROUP
// ====================================================
//
// EVENT_ACTIVE:
//     Producer: MotionTask
//     Consumers: DisplayTask, InputTask, AlarmTask
//     Set: PIR detects motion
//     Clear: 15-second inactivity timeout
//
// EVENT_MOTION:
//     Producer: MotionTask
//     Consumer: DisplayTask
//     Set: PIR detects motion
//     Clear: 15-second inactivity timeout
//
// EVENT_ALARM:
//     Producer: AlarmTask
//     Consumer: DisplayTask
//     Set: LOW or HIGH temperature
//     Clear: NORMAL temperature
//
// ====================================================

#define EVENT_ACTIVE BIT0
#define EVENT_MOTION BIT1
#define EVENT_ALARM  BIT2

EventGroupHandle_t systemEvents;

// ====================================================
// SENSOR DATA
// ====================================================

struct SensorData
{
    float temperature;
    float humidity;
    int lightLevel;
    bool motionDetected;
};

// ====================================================
// ALARM STRING HELPER
// ====================================================

static const char *alarmStateToString(AlarmState state)
{
    switch (state)
    {
        case AlarmState::NORMAL:
            return "NORMAL";

        case AlarmState::LOW_TEMPERATURE:
            return "LOW_TEMPERATURE";

        case AlarmState::HIGH_TEMPERATURE:
            return "HIGH_TEMPERATURE";
    }

    return "UNKNOWN";
}

// ====================================================
// DISPLAY MODE HELPERS
// ====================================================

static DisplayMode nextMode(DisplayMode mode)
{
    switch (mode)
    {
        case DisplayMode::TEMPERATURE:
            return DisplayMode::HUMIDITY;

        case DisplayMode::HUMIDITY:
            return DisplayMode::LIGHT;

        case DisplayMode::LIGHT:
            return DisplayMode::MOTION;

        case DisplayMode::MOTION:
            return DisplayMode::TEMPERATURE;
    }

    return DisplayMode::TEMPERATURE;
}

static DisplayMode previousMode(DisplayMode mode)
{
    switch (mode)
    {
        case DisplayMode::TEMPERATURE:
            return DisplayMode::MOTION;

        case DisplayMode::HUMIDITY:
            return DisplayMode::TEMPERATURE;

        case DisplayMode::LIGHT:
            return DisplayMode::HUMIDITY;

        case DisplayMode::MOTION:
            return DisplayMode::LIGHT;
    }

    return DisplayMode::TEMPERATURE;
}

static const char *modeToString(DisplayMode mode)
{
    switch (mode)
    {
        case DisplayMode::TEMPERATURE:
            return "TEMPERATURE";

        case DisplayMode::HUMIDITY:
            return "HUMIDITY";

        case DisplayMode::LIGHT:
            return "LIGHT";

        case DisplayMode::MOTION:
            return "MOTION";
    }

    return "UNKNOWN";
}

// ====================================================
// QUEUES
// ====================================================

QueueHandle_t displayQueue;
QueueHandle_t alarmQueue;
QueueHandle_t modeQueue;
QueueHandle_t encoderQueue;

// +1 = clockwise
// -1 = counter-clockwise

// ====================================================
// SYSTEM STATE
// ====================================================

volatile SystemState systemState =
    SystemState::ACTIVE;

volatile bool motionDetected = false;

// ====================================================
// HELPERS
// ====================================================

static inline TickType_t ms_to_ticks(uint32_t ms)
{
    TickType_t ticks = pdMS_TO_TICKS(ms);

    return (ticks == 0) ? 1 : ticks;
}

// ====================================================
// OLED
// ====================================================

static bool i2cErrorLogged = false;

static void report_i2c_error(esp_err_t err)
{
    if (err != ESP_OK && !i2cErrorLogged)
    {
        i2cErrorLogged = true;

        printf(
            "OLED I2C error: %s "
            "(check SDA=21, SCL=22, address 0x3C)\n",
            esp_err_to_name(err)
        );
    }
}

static void oled_command(uint8_t command)
{
    uint8_t data[2] = {0x00, command};

    esp_err_t err =
        i2c_master_write_to_device(
            I2C_PORT,
            OLED_ADDR,
            data,
            sizeof(data),
            pdMS_TO_TICKS(100)
        );

    report_i2c_error(err);
}

static void oled_data(
    const uint8_t *data,
    size_t length
)
{
    uint8_t buffer[129];

    if (length > 128)
        return;

    buffer[0] = 0x40;

    for (size_t i = 0; i < length; i++)
        buffer[i + 1] = data[i];

    esp_err_t err =
        i2c_master_write_to_device(
            I2C_PORT,
            OLED_ADDR,
            buffer,
            length + 1,
            pdMS_TO_TICKS(100)
        );

    report_i2c_error(err);
}

static void oled_init()
{
    i2c_config_t config = {};

    config.mode = I2C_MODE_MASTER;
    config.sda_io_num = OLED_SDA;
    config.scl_io_num = OLED_SCL;
    config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    config.master.clk_speed = 400000;

    esp_err_t err =
        i2c_param_config(
            I2C_PORT,
            &config
        );

    if (err != ESP_OK)
    {
        printf(
            "i2c_param_config failed: %s\n",
            esp_err_to_name(err)
        );
    }

    err =
        i2c_driver_install(
            I2C_PORT,
            config.mode,
            0,
            0,
            0
        );

    if (err != ESP_OK)
    {
        printf(
            "i2c_driver_install failed: %s\n",
            esp_err_to_name(err)
        );
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    oled_command(0xAE);
    oled_command(0xD5);
    oled_command(0x80);
    oled_command(0xA8);
    oled_command(0x3F);
    oled_command(0xD3);
    oled_command(0x00);
    oled_command(0x40);
    oled_command(0x8D);
    oled_command(0x14);
    oled_command(0x20);
    oled_command(0x00);
    oled_command(0xA1);
    oled_command(0xC8);
    oled_command(0xDA);
    oled_command(0x12);
    oled_command(0x81);
    oled_command(0xCF);
    oled_command(0xD9);
    oled_command(0xF1);
    oled_command(0xDB);
    oled_command(0x40);
    oled_command(0xA4);
    oled_command(0xA6);
    oled_command(0xAF);
}

static void oled_power(bool on)
{
    oled_command(on ? 0xAF : 0xAE);
}

static void oled_clear()
{
    uint8_t blank[128] = {0};

    for (int page = 0; page < 8; page++)
    {
        oled_command(0xB0 + page);
        oled_command(0x00);
        oled_command(0x10);
        oled_data(blank, 128);
    }
}

static void oled_set_cursor(
    int x,
    int page
)
{
    oled_command(0xB0 + page);
    oled_command(0x00 + (x & 0x0F));
    oled_command(0x10 + ((x >> 4) & 0x0F));
}

// ====================================================
// OLED CHARACTER FONT
// ====================================================

static void get_glyph(char c, uint8_t p[6])
{
    for (int i = 0; i < 6; i++)
        p[i] = 0;

    switch (c)
    {
        case 'A': p[0]=0x7E; p[1]=0x11; p[2]=0x11; p[3]=0x11; p[4]=0x7E; break;
        case 'B': p[0]=0x7F; p[1]=0x49; p[2]=0x49; p[3]=0x49; p[4]=0x36; break;
        case 'C': p[0]=0x3E; p[1]=0x41; p[2]=0x41; p[3]=0x41; p[4]=0x22; break;
        case 'D': p[0]=0x7F; p[1]=0x41; p[2]=0x41; p[3]=0x22; p[4]=0x1C; break;
        case 'E': p[0]=0x7F; p[1]=0x49; p[2]=0x49; p[3]=0x49; p[4]=0x41; break;
        case 'F': p[0]=0x7F; p[1]=0x09; p[2]=0x09; p[3]=0x09; p[4]=0x01; break;
        case 'G': p[0]=0x3E; p[1]=0x41; p[2]=0x49; p[3]=0x49; p[4]=0x7A; break;
        case 'H': p[0]=0x7F; p[1]=0x08; p[2]=0x08; p[3]=0x08; p[4]=0x7F; break;
        case 'I': p[0]=0x00; p[1]=0x41; p[2]=0x7F; p[3]=0x41; p[4]=0x00; break;
        case 'J': p[0]=0x20; p[1]=0x40; p[2]=0x41; p[3]=0x3F; p[4]=0x01; break;
        case 'K': p[0]=0x7F; p[1]=0x08; p[2]=0x14; p[3]=0x22; p[4]=0x41; break;
        case 'L': p[0]=0x7F; p[1]=0x40; p[2]=0x40; p[3]=0x40; p[4]=0x40; break;
        case 'M': p[0]=0x7F; p[1]=0x02; p[2]=0x0C; p[3]=0x02; p[4]=0x7F; break;
        case 'N': p[0]=0x7F; p[1]=0x02; p[2]=0x0C; p[3]=0x18; p[4]=0x7F; break;
        case 'O': p[0]=0x3E; p[1]=0x41; p[2]=0x41; p[3]=0x41; p[4]=0x3E; break;
        case 'P': p[0]=0x7F; p[1]=0x09; p[2]=0x09; p[3]=0x09; p[4]=0x06; break;
        case 'Q': p[0]=0x3E; p[1]=0x41; p[2]=0x51; p[3]=0x21; p[4]=0x5E; break;
        case 'R': p[0]=0x7F; p[1]=0x09; p[2]=0x19; p[3]=0x29; p[4]=0x46; break;
        case 'S': p[0]=0x46; p[1]=0x49; p[2]=0x49; p[3]=0x49; p[4]=0x31; break;
        case 'T': p[0]=0x01; p[1]=0x01; p[2]=0x7F; p[3]=0x01; p[4]=0x01; break;
        case 'U': p[0]=0x3F; p[1]=0x40; p[2]=0x40; p[3]=0x40; p[4]=0x3F; break;
        case 'V': p[0]=0x1F; p[1]=0x20; p[2]=0x40; p[3]=0x20; p[4]=0x1F; break;
        case 'W': p[0]=0x7F; p[1]=0x20; p[2]=0x18; p[3]=0x20; p[4]=0x7F; break;
        case 'X': p[0]=0x63; p[1]=0x14; p[2]=0x08; p[3]=0x14; p[4]=0x63; break;
        case 'Y': p[0]=0x07; p[1]=0x08; p[2]=0x70; p[3]=0x08; p[4]=0x07; break;
        case 'Z': p[0]=0x61; p[1]=0x51; p[2]=0x49; p[3]=0x45; p[4]=0x43; break;

        case '0': p[0]=0x3E; p[1]=0x51; p[2]=0x49; p[3]=0x45; p[4]=0x3E; break;
        case '1': p[0]=0x00; p[1]=0x42; p[2]=0x7F; p[3]=0x40; p[4]=0x00; break;
        case '2': p[0]=0x42; p[1]=0x61; p[2]=0x51; p[3]=0x49; p[4]=0x46; break;
        case '3': p[0]=0x21; p[1]=0x41; p[2]=0x45; p[3]=0x4B; p[4]=0x31; break;
        case '4': p[0]=0x18; p[1]=0x14; p[2]=0x12; p[3]=0x7F; p[4]=0x10; break;
        case '5': p[0]=0x27; p[1]=0x45; p[2]=0x45; p[3]=0x45; p[4]=0x39; break;
        case '6': p[0]=0x3C; p[1]=0x4A; p[2]=0x49; p[3]=0x49; p[4]=0x30; break;
        case '7': p[0]=0x01; p[1]=0x71; p[2]=0x09; p[3]=0x05; p[4]=0x03; break;
        case '8': p[0]=0x36; p[1]=0x49; p[2]=0x49; p[3]=0x49; p[4]=0x36; break;
        case '9': p[0]=0x06; p[1]=0x49; p[2]=0x49; p[3]=0x29; p[4]=0x1E; break;

        case '%': p[0]=0x63; p[1]=0x13; p[2]=0x08; p[3]=0x64; p[4]=0x63; break;
        case '.': p[0]=0x00; p[1]=0x60; p[2]=0x60; p[3]=0x00; p[4]=0x00; break;
        case '-': p[0]=0x08; p[1]=0x08; p[2]=0x08; p[3]=0x08; p[4]=0x08; break;
        case ' ': break;

        default: break;
    }
}

static void oled_char(
    int x,
    int page,
    char c
)
{
    uint8_t p[6];

    get_glyph(c, p);

    oled_set_cursor(x, page);
    oled_data(p, 6);
}

static void oled_text(
    int x,
    int page,
    const char *text
)
{
    while (*text)
    {
        oled_char(x, page, *text);
        x += 6;
        text++;
    }
}

static void oled_char_big(
    int x,
    int page,
    char c
)
{
    uint8_t g[6];
    uint8_t top[12];
    uint8_t bottom[12];

    get_glyph(c, g);

    for (int i = 0; i < 6; i++)
    {
        uint8_t low = 0;
        uint8_t high = 0;

        for (int b = 0; b < 4; b++)
        {
            if (g[i] & (1 << b))
                low |= (uint8_t)(3 << (2 * b));

            if (g[i] & (1 << (b + 4)))
                high |= (uint8_t)(3 << (2 * b));
        }

        top[2 * i] = low;
        top[2 * i + 1] = low;

        bottom[2 * i] = high;
        bottom[2 * i + 1] = high;
    }

    oled_set_cursor(x, page);
    oled_data(top, 12);

    oled_set_cursor(x, page + 1);
    oled_data(bottom, 12);
}

static void oled_text_big(
    int x,
    int page,
    const char *text
)
{
    while (*text)
    {
        oled_char_big(x, page, *text);
        x += 12;
        text++;
    }
}

// ====================================================
// DHT22
// ====================================================

static portMUX_TYPE dhtMux =
    portMUX_INITIALIZER_UNLOCKED;

static bool dht22_read(
    float *temperature,
    float *humidity
)
{
    uint8_t data[5] = {0, 0, 0, 0, 0};

    bool success = false;

    portENTER_CRITICAL(&dhtMux);

    gpio_set_direction(
        DHT_PIN,
        GPIO_MODE_OUTPUT
    );

    gpio_set_level(
        DHT_PIN,
        0
    );

    esp_rom_delay_us(1200);

    gpio_set_level(
        DHT_PIN,
        1
    );

    esp_rom_delay_us(30);

    gpio_set_direction(
        DHT_PIN,
        GPIO_MODE_INPUT
    );

    gpio_pullup_en(DHT_PIN);

    int64_t start =
        esp_timer_get_time();

    while (gpio_get_level(DHT_PIN) == 1)
    {
        if (
            (esp_timer_get_time() - start)
            > DHT_TIMEOUT_US
        )
        {
            break;
        }
    }

    bool responseLow =
        (gpio_get_level(DHT_PIN) == 0);

    if (responseLow)
    {
        start =
            esp_timer_get_time();

        while (gpio_get_level(DHT_PIN) == 0)
        {
            if (
                (esp_timer_get_time() - start)
                > DHT_TIMEOUT_US
            )
            {
                break;
            }
        }
    }

    bool responseHigh =
        (gpio_get_level(DHT_PIN) == 1);

    if (responseLow && responseHigh)
    {
        start =
            esp_timer_get_time();

        while (gpio_get_level(DHT_PIN) == 1)
        {
            if (
                (esp_timer_get_time() - start)
                > DHT_TIMEOUT_US
            )
            {
                break;
            }
        }

        bool firstBitStarted =
            (gpio_get_level(DHT_PIN) == 0);

        if (firstBitStarted)
        {
            bool readOK = true;

            for (int i = 0; i < 40; i++)
            {
                start =
                    esp_timer_get_time();

                while (gpio_get_level(DHT_PIN) == 0)
                {
                    if (
                        (esp_timer_get_time() - start)
                        > DHT_BIT_TIMEOUT_US
                    )
                    {
                        readOK = false;
                        break;
                    }
                }

                if (!readOK)
                    break;

                int64_t highStart =
                    esp_timer_get_time();

                while (gpio_get_level(DHT_PIN) == 1)
                {
                    if (
                        (esp_timer_get_time() - highStart)
                        > DHT_BIT_TIMEOUT_US
                    )
                    {
                        readOK = false;
                        break;
                    }
                }

                if (!readOK)
                    break;

                int64_t pulseLength =
                    esp_timer_get_time()
                    - highStart;

                int byteIndex =
                    i / 8;

                int bitIndex =
                    7 - (i % 8);

                if (
                    pulseLength
                    > DHT_ONE_THRESHOLD_US
                )
                {
                    data[byteIndex] |=
                        (uint8_t)(
                            1U << bitIndex
                        );
                }
            }

            if (readOK)
            {
                uint8_t checksum =
                    (uint8_t)(
                        data[0] +
                        data[1] +
                        data[2] +
                        data[3]
                    );

                if (checksum == data[4])
                {
                    uint16_t rawHumidity =
                        ((uint16_t)data[0] << 8)
                        | data[1];

                    uint16_t rawTemperature =
                        ((uint16_t)data[2] << 8)
                        | data[3];

                    float newHumidity =
                        rawHumidity / 10.0f;

                    bool negative =
                        (rawTemperature & 0x8000U)
                        != 0;

                    rawTemperature &=
                        0x7FFFU;

                    float newTemperature =
                        rawTemperature / 10.0f;

                    if (negative)
                        newTemperature =
                            -newTemperature;

                    if (
                        newHumidity >= 0.0f &&
                        newHumidity <= 100.0f &&
                        newTemperature >= -40.0f &&
                        newTemperature <= 80.0f
                    )
                    {
                        *humidity =
                            newHumidity;

                        *temperature =
                            newTemperature;

                        success = true;
                    }
                    else
                    {
                        printf(
                            "DHT22 invalid values: "
                            "T=%.2f C H=%.2f %%\n",
                            newTemperature,
                            newHumidity
                        );
                    }
                }
                else
                {
                    printf(
                        "DHT22 checksum failed "
                        "(calc=%u received=%u)\n",
                        checksum,
                        data[4]
                    );
                }
            }
            else
            {
                printf(
                    "DHT22 timing/read failed\n"
                );
            }
        }
    }

    if (
        !success &&
        !(responseLow && responseHigh)
    )
    {
        printf(
            "DHT22 sensor response timeout\n"
        );
    }

    gpio_set_direction(
        DHT_PIN,
        GPIO_MODE_INPUT
    );

    gpio_pullup_en(DHT_PIN);

    portEXIT_CRITICAL(&dhtMux);

    return success;
}

// ====================================================
// SENSOR TASK
// ====================================================

void sensorTask(void *parameter)
{
    SensorData sensorData = {};

    TickType_t lastWakeTime =
        xTaskGetTickCount();

    adc_oneshot_unit_handle_t adc_handle =
        NULL;

    adc_oneshot_unit_init_cfg_t init_config = {};

    init_config.unit_id =
        ADC_UNIT_1;

    init_config.clk_src =
        ADC_RTC_CLK_SRC_DEFAULT;

    init_config.ulp_mode =
        ADC_ULP_MODE_DISABLE;

    esp_err_t result =
        adc_oneshot_new_unit(
            &init_config,
            &adc_handle
        );

    if (result != ESP_OK)
    {
        printf(
            "LDR ADC initialization failed: %s\n",
            esp_err_to_name(result)
        );

        vTaskDelete(NULL);
        return;
    }

    adc_oneshot_chan_cfg_t channel_config = {};

    channel_config.atten =
        ADC_ATTEN_DB_12;

    channel_config.bitwidth =
        ADC_BITWIDTH_12;

    result =
        adc_oneshot_config_channel(
            adc_handle,
            LDR_ADC_CHANNEL,
            &channel_config
        );

    if (result != ESP_OK)
    {
        printf(
            "LDR ADC channel configuration failed: %s\n",
            esp_err_to_name(result)
        );

        vTaskDelete(NULL);
        return;
    }

    sensorData.temperature = 0.0f;
    sensorData.humidity = 0.0f;
    sensorData.lightLevel = 0;
    sensorData.motionDetected = false;

    bool haveValidDhtReading = false;

    printf(
        "SensorTask: Started on CPU %d\n",
        xPortGetCoreID()
    );

    while (1)
    {
        float newTemperature = 0.0f;
        float newHumidity = 0.0f;

        if (
            dht22_read(
                &newTemperature,
                &newHumidity
            )
        )
        {
            sensorData.temperature =
                newTemperature;

            sensorData.humidity =
                newHumidity;

            haveValidDhtReading =
                true;
        }
        else
        {
            if (haveValidDhtReading)
            {
                printf(
                    "DHT22 reading failed - "
                    "keeping previous valid reading\n"
                );
            }
            else
            {
                printf(
                    "DHT22 reading failed - "
                    "no valid reading yet\n"
                );
            }
        }

        int raw_value = 0;

        result =
            adc_oneshot_read(
                adc_handle,
                LDR_ADC_CHANNEL,
                &raw_value
            );

        if (result == ESP_OK)
        {
            sensorData.lightLevel =
                (int)(
                    (raw_value / 4095.0f)
                    * 100.0f
                );

            if (sensorData.lightLevel < 0)
                sensorData.lightLevel = 0;

            if (sensorData.lightLevel > 100)
                sensorData.lightLevel = 100;
        }
        else
        {
            sensorData.lightLevel = 0;
        }

        sensorData.motionDetected =
            motionDetected;

        printf(
            "Temperature: %.2f C\n",
            sensorData.temperature
        );

        printf(
            "Humidity: %.2f %%\n",
            sensorData.humidity
        );

        printf(
            "Light Level: %d %%\n",
            sensorData.lightLevel
        );

        printf(
            "Motion: %s\n",
            sensorData.motionDetected
                ? "DETECTED"
                : "NONE"
        );

        if (haveValidDhtReading)
        {
            xQueueOverwrite(
                displayQueue,
                &sensorData
            );

            xQueueOverwrite(
                alarmQueue,
                &sensorData
            );

            printf(
                "Sensor data sent to queues\n"
            );
        }

        vTaskDelayUntil(
            &lastWakeTime,
            pdMS_TO_TICKS(2000)
        );
    }
}

// ====================================================
// MOTION TASK
// ====================================================

void motionTask(void *parameter)
{
    gpio_set_direction(
        PIR_PIN,
        GPIO_MODE_INPUT
    );

    gpio_pulldown_en(PIR_PIN);

    int64_t lastMotionTime =
        esp_timer_get_time();

    bool lastPirLevel = false;

    printf(
        "MotionTask: Started on CPU %d\n",
        xPortGetCoreID()
    );

    while (1)
    {
        bool pirLevel =
            gpio_get_level(PIR_PIN) != 0;

        int64_t now =
            esp_timer_get_time();

        if (pirLevel)
        {
            lastMotionTime = now;

            motionDetected = true;

            // Part X: set ACTIVE + MOTION events
            xEventGroupSetBits(
                systemEvents,
                EVENT_ACTIVE | EVENT_MOTION
            );

            if (!lastPirLevel)
            {
                printf(
                    "MotionTask: MOTION DETECTED\n"
                );
            }

            if (
                systemState ==
                SystemState::INACTIVE
            )
            {
                systemState =
                    SystemState::ACTIVE;

                printf(
                    "System entering ACTIVE mode\n"
                );
            }
        }

        lastPirLevel =
            pirLevel;

        if (
            systemState == SystemState::ACTIVE &&
            (now - lastMotionTime)
                >= INACTIVITY_TIMEOUT_US
        )
        {
            motionDetected = false;

            systemState =
                SystemState::INACTIVE;

            // Part X: clear ACTIVE + MOTION events
            xEventGroupClearBits(
                systemEvents,
                EVENT_ACTIVE | EVENT_MOTION
            );

            printf(
                "MotionTask: NO MOTION - "
                "15 SECOND TIMEOUT\n"
            );

            printf(
                "System entering INACTIVE mode\n"
            );
        }

        vTaskDelay(
            ms_to_ticks(100)
        );
    }
}

// ====================================================
// ENCODER INTERRUPT
// ====================================================

static void IRAM_ATTR encoderIsr(void *arg)
{
    int8_t direction =
        (gpio_get_level(ENCODER_DT) != 0)
            ? 1
            : -1;

    if (ENCODER_REVERSE)
        direction = -direction;

    BaseType_t higherPriorityTaskWoken =
        pdFALSE;

    xQueueSendFromISR(
        encoderQueue,
        &direction,
        &higherPriorityTaskWoken
    );

    if (higherPriorityTaskWoken == pdTRUE)
    {
        portYIELD_FROM_ISR();
    }
}

// ====================================================
// INPUT TASK
// ====================================================

void inputTask(void *parameter)
{
    gpio_set_direction(
        ENCODER_CLK,
        GPIO_MODE_INPUT
    );

    gpio_set_direction(
        ENCODER_DT,
        GPIO_MODE_INPUT
    );

    gpio_set_direction(
        ENCODER_SW,
        GPIO_MODE_INPUT
    );

    gpio_pullup_en(ENCODER_CLK);
    gpio_pullup_en(ENCODER_DT);
    gpio_pullup_en(ENCODER_SW);

    gpio_set_intr_type(
        ENCODER_CLK,
        GPIO_INTR_NEGEDGE
    );

    esp_err_t err =
        gpio_install_isr_service(0);

    if (
        err != ESP_OK &&
        err != ESP_ERR_INVALID_STATE
    )
    {
        printf(
            "gpio_install_isr_service failed: %s\n",
            esp_err_to_name(err)
        );
    }

    gpio_isr_handler_add(
        ENCODER_CLK,
        encoderIsr,
        NULL
    );

    DisplayMode currentMode =
        DisplayMode::TEMPERATURE;

    bool lastButton =
        gpio_get_level(ENCODER_SW);

    printf(
        "InputTask: Started on CPU %d\n",
        xPortGetCoreID()
    );

    while (1)
    {
        int8_t direction = 0;

        if (
            xQueueReceive(
                encoderQueue,
                &direction,
                ms_to_ticks(20)
            ) == pdPASS
        )
        {
            // Part X: read ACTIVE event
            EventBits_t events =
                xEventGroupGetBits(systemEvents);

            bool active =
                (events & EVENT_ACTIVE) != 0;

            if (active)
            {
                if (direction > 0)
                {
                    currentMode =
                        nextMode(currentMode);

                    printf(
                        "InputTask: clockwise -> %s\n",
                        modeToString(currentMode)
                    );
                }
                else
                {
                    currentMode =
                        previousMode(currentMode);

                    printf(
                        "InputTask: counter-clockwise -> %s\n",
                        modeToString(currentMode)
                    );
                }

                xQueueOverwrite(
                    modeQueue,
                    &currentMode
                );
            }
        }

        bool currentButton =
            gpio_get_level(ENCODER_SW);

        // Read ACTIVE event before accepting button input
        EventBits_t events =
            xEventGroupGetBits(systemEvents);

        bool active =
            (events & EVENT_ACTIVE) != 0;

        if (
            lastButton == 1 &&
            currentButton == 0 &&
            active
        )
        {
            currentMode =
                nextMode(currentMode);

            xQueueOverwrite(
                modeQueue,
                &currentMode
            );

            printf(
                "InputTask: button pressed -> %s\n",
                modeToString(currentMode)
            );

            vTaskDelay(
                ms_to_ticks(200)
            );
        }

        lastButton =
            currentButton;
    }
}

// ====================================================
// DISPLAY PAGE DRAWING
// ====================================================

static void drawPage(
    DisplayMode mode,
    const SensorData &data
)
{
    char line[16];

    oled_clear();

    oled_text(
        0,
        0,
        "ROOM MONITOR"
    );

    switch (mode)
    {
        case DisplayMode::TEMPERATURE:

            oled_text_big(
                0,
                2,
                "TEMP"
            );

            snprintf(
                line,
                sizeof(line),
                "%.1f C",
                data.temperature
            );

            oled_text_big(
                0,
                5,
                line
            );

            printf(
                "DisplayTask: "
                "Temperature page = %.2f C\n",
                data.temperature
            );

            break;

        case DisplayMode::HUMIDITY:

            oled_text_big(
                0,
                2,
                "HUMIDITY"
            );

            snprintf(
                line,
                sizeof(line),
                "%d %%",
                (int)data.humidity
            );

            oled_text_big(
                0,
                5,
                line
            );

            printf(
                "DisplayTask: "
                "Humidity page = %.2f %%\n",
                data.humidity
            );

            break;

        case DisplayMode::LIGHT:

            oled_text_big(
                0,
                2,
                "LIGHT"
            );

            snprintf(
                line,
                sizeof(line),
                "%d %%",
                data.lightLevel
            );

            oled_text_big(
                0,
                5,
                line
            );

            printf(
                "DisplayTask: "
                "Light page = %d %%\n",
                data.lightLevel
            );

            break;

        case DisplayMode::MOTION:

            oled_text_big(
                0,
                2,
                "MOTION"
            );

            oled_text_big(
                0,
                5,
                motionDetected
                    ? "DETECTED"
                    : "NONE"
            );

            printf(
                "DisplayTask: Motion page = %s\n",
                motionDetected
                    ? "DETECTED"
                    : "NONE"
            );

            break;
    }
}

// ====================================================
// DISPLAY TASK
// ====================================================

void displayTask(void *parameter)
{
    SensorData sensorData = {};

    DisplayMode currentMode =
        DisplayMode::TEMPERATURE;

    bool haveData = false;
    bool displayOn = true;
    bool needRedraw = true;
    bool lastMotionShown = false;
    bool lastAlarmShown = false;

    oled_init();
    oled_clear();

    printf(
        "DisplayTask: Started on CPU %d\n",
        xPortGetCoreID()
    );

    while (1)
    {
        if (
            xQueueReceive(
                displayQueue,
                &sensorData,
                ms_to_ticks(20)
            ) == pdPASS
        )
        {
            haveData = true;
            needRedraw = true;
        }

        DisplayMode newMode;

        if (
            xQueueReceive(
                modeQueue,
                &newMode,
                0
            ) == pdPASS
        )
        {
            currentMode = newMode;
            needRedraw = true;
        }

        // --------------------------------------------
        // PART X - READ EVENT GROUP
        // --------------------------------------------

        EventBits_t events =
            xEventGroupGetBits(systemEvents);

        bool active =
            (events & EVENT_ACTIVE) != 0;

        bool motion =
            (events & EVENT_MOTION) != 0;

        bool alarm =
            (events & EVENT_ALARM) != 0;

        // --------------------------------------------
        // INACTIVE
        // --------------------------------------------

        if (!active)
        {
            if (displayOn)
            {
                oled_clear();
                oled_power(false);

                displayOn = false;

                printf(
                    "DisplayTask: OLED OFF - "
                    "system INACTIVE\n"
                );
            }

            continue;
        }

        // --------------------------------------------
        // ACTIVE
        // --------------------------------------------

        if (!displayOn)
        {
            oled_power(true);

            displayOn = true;
            needRedraw = true;

            printf(
                "DisplayTask: OLED ON - "
                "system ACTIVE\n"
            );
        }

        if (
            currentMode == DisplayMode::MOTION &&
            motion != lastMotionShown
        )
        {
            needRedraw = true;
        }

        // Alarm event is consumed here.
        if (alarm != lastAlarmShown)
        {
            lastAlarmShown = alarm;

            printf(
                "DisplayTask: Alarm event = %s\n",
                alarm ? "ACTIVE" : "CLEAR"
            );
        }

        if (!haveData || !needRedraw)
            continue;

        needRedraw = false;
        lastMotionShown = motion;

        drawPage(
            currentMode,
            sensorData
        );
    }
}

// ====================================================
// ALARM TASK
// ====================================================

void alarmTask(void *parameter)
{
    SensorData sensorData = {};

    AlarmState lastState =
        AlarmState::NORMAL;

    bool firstEvaluation = true;

    printf(
        "AlarmTask: Started on CPU %d\n",
        xPortGetCoreID()
    );

    while (1)
    {
        if (
            xQueueReceive(
                alarmQueue,
                &sensorData,
                portMAX_DELAY
            ) == pdPASS
        )
        {
            // ----------------------------------------
            // READ ACTIVE EVENT
            // ----------------------------------------

            EventBits_t events =
                xEventGroupGetBits(systemEvents);

            bool active =
                (events & EVENT_ACTIVE) != 0;

            if (!active)
            {
                xEventGroupClearBits(
                    systemEvents,
                    EVENT_ALARM
                );

                firstEvaluation = true;

                continue;
            }

            // ----------------------------------------
            // TEMPERATURE DECISION
            // ----------------------------------------

            AlarmState state =
                evaluateTemperature(
                    sensorData.temperature
                );

            // ----------------------------------------
            // SET/CLEAR ALARM EVENT
            // ----------------------------------------

            if (state != AlarmState::NORMAL)
            {
                xEventGroupSetBits(
                    systemEvents,
                    EVENT_ALARM
                );
            }
            else
            {
                xEventGroupClearBits(
                    systemEvents,
                    EVENT_ALARM
                );
            }

            // ----------------------------------------
            // LOG STATE CHANGE
            // ----------------------------------------

            if (
                firstEvaluation ||
                state != lastState
            )
            {
                printf(
                    "Alarm State: %s (%.1f C)\n",
                    alarmStateToString(state),
                    sensorData.temperature
                );

                // Buzzer hardware control would go here.

                lastState = state;
                firstEvaluation = false;
            }
        }
    }
}

// ====================================================
// MAIN
// ====================================================

extern "C" void app_main(void)
{
    printf("\n");

    printf(
        "====================================\n"
    );

    printf(
        "BCA152 FreeRTOS Multisensor\n"
    );

    printf(
        "System starting...\n"
    );

    printf(
        "====================================\n"
    );

    // --------------------------------------------
    // DHT22 IDLE STATE
    // --------------------------------------------

    gpio_set_direction(
        DHT_PIN,
        GPIO_MODE_INPUT
    );

    gpio_pullup_en(DHT_PIN);

    // --------------------------------------------
    // PIR
    // --------------------------------------------

    gpio_set_direction(
        PIR_PIN,
        GPIO_MODE_INPUT
    );

    gpio_pulldown_en(PIR_PIN);

    // --------------------------------------------
    // CREATE QUEUES
    // --------------------------------------------

    displayQueue =
        xQueueCreate(
            1,
            sizeof(SensorData)
        );

    alarmQueue =
        xQueueCreate(
            1,
            sizeof(SensorData)
        );

    modeQueue =
        xQueueCreate(
            1,
            sizeof(DisplayMode)
        );

    encoderQueue =
        xQueueCreate(
            8,
            sizeof(int8_t)
        );

    if (
        displayQueue == NULL ||
        alarmQueue == NULL ||
        modeQueue == NULL ||
        encoderQueue == NULL
    )
    {
        printf(
            "ERROR: Failed to create queues\n"
        );

        return;
    }

    printf(
        "Queues created successfully.\n"
    );

    // --------------------------------------------
    // CREATE EVENT GROUP
    // --------------------------------------------

    systemEvents =
        xEventGroupCreate();

    if (systemEvents == NULL)
    {
        printf(
            "ERROR: Failed to create event group\n"
        );

        return;
    }

    printf(
        "Event group created successfully.\n"
    );

    // Initial system state is ACTIVE.
    xEventGroupSetBits(
        systemEvents,
        EVENT_ACTIVE
    );

    printf(
        "Initial event: EVENT_ACTIVE\n"
    );

    // --------------------------------------------
    // SENSOR TASK - CPU0
    // --------------------------------------------

    BaseType_t result =
        xTaskCreatePinnedToCore(
            sensorTask,
            "SensorTask",
            6144,
            NULL,
            2,
            NULL,
            0
        );

    if (result != pdPASS)
    {
        printf(
            "ERROR: SensorTask creation failed\n"
        );

        return;
    }

    // --------------------------------------------
    // MOTION TASK - CPU0
    // --------------------------------------------

    result =
        xTaskCreatePinnedToCore(
            motionTask,
            "MotionTask",
            4096,
            NULL,
            2,
            NULL,
            0
        );

    if (result != pdPASS)
    {
        printf(
            "ERROR: MotionTask creation failed\n"
        );

        return;
    }

    // --------------------------------------------
    // INPUT TASK - CPU1
    // --------------------------------------------

    result =
        xTaskCreatePinnedToCore(
            inputTask,
            "InputTask",
            4096,
            NULL,
            2,
            NULL,
            1
        );

    if (result != pdPASS)
    {
        printf(
            "ERROR: InputTask creation failed\n"
        );

        return;
    }

    // --------------------------------------------
    // DISPLAY TASK - CPU1
    // --------------------------------------------

    result =
        xTaskCreatePinnedToCore(
            displayTask,
            "DisplayTask",
            4096,
            NULL,
            1,
            NULL,
            1
        );

    if (result != pdPASS)
    {
        printf(
            "ERROR: DisplayTask creation failed\n"
        );

        return;
    }

    // --------------------------------------------
    // ALARM TASK - CPU1
    // --------------------------------------------

    result =
        xTaskCreatePinnedToCore(
            alarmTask,
            "AlarmTask",
            4096,
            NULL,
            1,
            NULL,
            1
        );

    if (result != pdPASS)
    {
        printf(
            "ERROR: AlarmTask creation failed\n"
        );

        return;
    }

    printf(
        "All tasks started successfully.\n"
    );

    printf(
        "Part X Event Group initialized.\n"
    );

    printf(
        "EVENT_ACTIVE = BIT0\n"
    );

    printf(
        "EVENT_MOTION = BIT1\n"
    );

    printf(
        "EVENT_ALARM  = BIT2\n"
    );
}