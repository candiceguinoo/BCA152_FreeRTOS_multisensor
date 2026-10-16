#include <stdio.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"

#include "display.h"
#include "input.h"
#include "rtos_objects.h"
#include "sensors.h"
#include "system_state.h"

// OLED / I2C (this module only)
#define OLED_SDA GPIO_NUM_21
#define OLED_SCL GPIO_NUM_22
#define OLED_ADDR 0x3C
#define I2C_PORT I2C_NUM_0

// ====================================================
// OLED
// ====================================================

static bool i2cErrorLogged = false;

static void report_i2c_error(esp_err_t err)
{
    if (err != ESP_OK && !i2cErrorLogged)
    {
        i2cErrorLogged = true;

        serialPrintf(
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

static void oled_data(const uint8_t *data, size_t length)
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

    esp_err_t err = i2c_param_config(I2C_PORT, &config);

    if (err != ESP_OK)
    {
        serialPrintf(
            "i2c_param_config failed: %s\n",
            esp_err_to_name(err)
        );
    }

    err = i2c_driver_install(
        I2C_PORT,
        config.mode,
        0,
        0,
        0
    );

    if (err != ESP_OK)
    {
        serialPrintf(
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

static void oled_set_cursor(int x, int page)
{
    oled_command(0xB0 + page);
    oled_command(0x00 + (x & 0x0F));
    oled_command(0x10 + ((x >> 4) & 0x0F));
}

// ====================================================
// OLED FONT
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

        default:
            break;
    }
}

static void oled_char(int x, int page, char c)
{
    uint8_t p[6];

    get_glyph(c, p);

    oled_set_cursor(x, page);
    oled_data(p, 6);
}

static void oled_text(int x, int page, const char *text)
{
    while (*text)
    {
        oled_char(x, page, *text);
        x += 6;
        text++;
    }
}

static void oled_char_big(int x, int page, char c)
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

static void oled_text_big(int x, int page, const char *text)
{
    while (*text)
    {
        oled_char_big(x, page, *text);
        x += 12;
        text++;
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

            serialPrintf(
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

            serialPrintf(
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

            serialPrintf(
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

            serialPrintf(
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

    oled_init();
    oled_clear();

    serialPrintf(
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
        // EVENT GROUP STATE
        // --------------------------------------------

        EventBits_t events =
            xEventGroupGetBits(systemEvents);

        bool active =
            (events & EVENT_ACTIVE) != 0;

        static EventBits_t previousEvents = 0;

        if (events != previousEvents)
        {
            serialPrintf(
                "DisplayTask Events: "
                "ACTIVE=%d MOTION=%d ALARM=%d\n",
                (events & EVENT_ACTIVE) ? 1 : 0,
                (events & EVENT_MOTION) ? 1 : 0,
                (events & EVENT_ALARM) ? 1 : 0
            );

            previousEvents = events;
        }

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

                serialPrintf(
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

            serialPrintf(
                "DisplayTask: OLED ON - "
                "system ACTIVE\n"
            );
        }

        if (
            currentMode == DisplayMode::MOTION &&
            motionDetected != lastMotionShown
        )
        {
            needRedraw = true;
        }

        if (!haveData || !needRedraw)
            continue;

        needRedraw = false;

        lastMotionShown =
            motionDetected;


        drawPage(
            currentMode,
            sensorData
        );

    }
}