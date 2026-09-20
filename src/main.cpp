#include <stdio.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "driver/i2c.h"

#include "esp_timer.h"
#include "esp_rom_sys.h"

#include "esp_adc/adc_oneshot.h"

#define DHT_PIN GPIO_NUM_4

// LDR is connected to GPIO 34
// GPIO 34 = ADC1_CHANNEL_6
#define LDR_CHANNEL ADC_CHANNEL_6

// ----------------------------------------------------
// OLED I2C Configuration
// ----------------------------------------------------
#define I2C_PORT I2C_NUM_0
#define I2C_SDA GPIO_NUM_21
#define I2C_SCL GPIO_NUM_22
#define OLED_ADDRESS 0x3C

// ----------------------------------------------------
// Sensor Data Structure
// ----------------------------------------------------
struct SensorData
{
    float temperature;
    float humidity;
    int lightLevel;
    bool motionDetected;
};

// Sensor Queue
QueueHandle_t sensorQueue;

// ----------------------------------------------------
// Simple OLED Font
// ----------------------------------------------------
static const uint8_t font_5x7[][5] = {
    // Space
    {0x00, 0x00, 0x00, 0x00, 0x00},

    // .
    {0x00, 0x00, 0x00, 0x60, 0x60},

    // 0
    {0x3E, 0x51, 0x49, 0x45, 0x3E},

    // 1
    {0x00, 0x42, 0x7F, 0x40, 0x00},

    // 2
    {0x42, 0x61, 0x51, 0x49, 0x46},

    // 3
    {0x21, 0x41, 0x45, 0x4B, 0x31},

    // 4
    {0x18, 0x14, 0x12, 0x7F, 0x10},

    // 5
    {0x27, 0x45, 0x45, 0x45, 0x39},

    // 6
    {0x3C, 0x4A, 0x49, 0x49, 0x30},

    // 7
    {0x01, 0x71, 0x09, 0x05, 0x03},

    // 8
    {0x36, 0x49, 0x49, 0x49, 0x36},

    // 9
    {0x06, 0x49, 0x49, 0x29, 0x1E},

    // A
    {0x7E, 0x09, 0x09, 0x09, 0x7E},

    // C
    {0x3E, 0x41, 0x41, 0x41, 0x22},

    // E
    {0x7F, 0x49, 0x49, 0x49, 0x41},

    // I
    {0x00, 0x41, 0x7F, 0x41, 0x00},

    // K
    {0x7F, 0x08, 0x14, 0x22, 0x41},

    // M
    {0x7F, 0x02, 0x0C, 0x02, 0x7F},

    // N
    {0x7F, 0x04, 0x08, 0x10, 0x7F},

    // O
    {0x3E, 0x41, 0x41, 0x41, 0x3E},

    // P
    {0x7F, 0x09, 0x09, 0x09, 0x06},

    // R
    {0x7F, 0x09, 0x19, 0x29, 0x46},

    // T
    {0x01, 0x01, 0x7F, 0x01, 0x01},

    // U
    {0x3F, 0x40, 0x40, 0x40, 0x3F}
};

// ----------------------------------------------------
// Get Font Character
// ----------------------------------------------------
static const uint8_t *getFont(char c)
{
    if (c == ' ')
        return font_5x7[0];

    if (c == '.')
        return font_5x7[1];

    if (c >= '0' && c <= '9')
        return font_5x7[2 + (c - '0')];

    switch (c)
    {
        case 'A': return font_5x7[12];
        case 'C': return font_5x7[13];
        case 'E': return font_5x7[14];
        case 'I': return font_5x7[15];
        case 'K': return font_5x7[16];
        case 'M': return font_5x7[17];
        case 'N': return font_5x7[18];
        case 'O': return font_5x7[19];
        case 'P': return font_5x7[20];
        case 'R': return font_5x7[21];
        case 'T': return font_5x7[22];
        case 'U': return font_5x7[23];
        default: return font_5x7[0];
    }
}

// ----------------------------------------------------
// OLED Write Command
// ----------------------------------------------------
static void oled_command(uint8_t command)
{
    uint8_t data[2];

    data[0] = 0x00;
    data[1] = command;

    i2c_master_write_to_device(
        I2C_PORT,
        OLED_ADDRESS,
        data,
        sizeof(data),
        pdMS_TO_TICKS(100)
    );
}

// ----------------------------------------------------
// OLED Write Data
// ----------------------------------------------------
static void oled_data(const uint8_t *data, size_t length)
{
    uint8_t buffer[129];

    buffer[0] = 0x40;

    for (size_t i = 0; i < length; i++)
    {
        buffer[i + 1] = data[i];
    }

    i2c_master_write_to_device(
        I2C_PORT,
        OLED_ADDRESS,
        buffer,
        length + 1,
        pdMS_TO_TICKS(100)
    );
}

// ----------------------------------------------------
// OLED Initialize
// ----------------------------------------------------
static void oled_init()
{
    i2c_config_t config = {};

    config.mode = I2C_MODE_MASTER;
    config.sda_io_num = I2C_SDA;
    config.scl_io_num = I2C_SCL;

    config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    config.scl_pullup_en = GPIO_PULLUP_ENABLE;

    config.master.clk_speed = 100000;

    i2c_param_config(I2C_PORT, &config);

    i2c_driver_install(
        I2C_PORT,
        I2C_MODE_MASTER,
        0,
        0,
        0
    );

    // SSD1306 initialization
    oled_command(0xAE); // Display OFF
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
    oled_command(0xAF); // Display ON
}

// ----------------------------------------------------
// Set OLED Position
// ----------------------------------------------------
static void oled_set_cursor(uint8_t page, uint8_t column)
{
    oled_command(0xB0 + page);
    oled_command(0x00 + (column & 0x0F));
    oled_command(0x10 + ((column >> 4) & 0x0F));
}

// ----------------------------------------------------
// Clear OLED
// ----------------------------------------------------
static void oled_clear()
{
    uint8_t blank[128] = {0};

    for (uint8_t page = 0; page < 8; page++)
    {
        oled_set_cursor(page, 0);
        oled_data(blank, 128);
    }
}

// ----------------------------------------------------
// Write One Character
// ----------------------------------------------------
static void oled_write_char(char c)
{
    const uint8_t *character = getFont(c);

    uint8_t data[6];

    for (int i = 0; i < 5; i++)
    {
        data[i] = character[i];
    }

    data[5] = 0x00;

    oled_data(data, 6);
}

// ----------------------------------------------------
// Write String
// ----------------------------------------------------
static void oled_write_string(const char *text)
{
    while (*text)
    {
        char c = *text;

        if (c >= 'a' && c <= 'z')
        {
            c = c - ('a' - 'A');
        }

        oled_write_char(c);

        text++;
    }
}

// ----------------------------------------------------
// DHT22 Reading Function
// ----------------------------------------------------
static bool dht22_read(float *temperature, float *humidity)
{
    uint8_t data[5] = {0, 0, 0, 0, 0};

    // Start signal
    gpio_set_direction(DHT_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(DHT_PIN, 0);
    esp_rom_delay_us(1200);

    gpio_set_level(DHT_PIN, 1);
    esp_rom_delay_us(30);

    // Listen for DHT22 response
    gpio_set_direction(DHT_PIN, GPIO_MODE_INPUT);
    gpio_pullup_en(DHT_PIN);

    // Wait for response LOW
    int64_t start = esp_timer_get_time();

    while (gpio_get_level(DHT_PIN) == 1)
    {
        if (esp_timer_get_time() - start > 100)
            return false;
    }

    // Wait for response HIGH
    start = esp_timer_get_time();

    while (gpio_get_level(DHT_PIN) == 0)
    {
        if (esp_timer_get_time() - start > 100)
            return false;
    }

    // Wait for response LOW
    start = esp_timer_get_time();

    while (gpio_get_level(DHT_PIN) == 1)
    {
        if (esp_timer_get_time() - start > 100)
            return false;
    }

    // Read 40 bits
    for (int i = 0; i < 40; i++)
    {
        start = esp_timer_get_time();

        while (gpio_get_level(DHT_PIN) == 0)
        {
            if (esp_timer_get_time() - start > 100)
                return false;
        }

        int64_t high_start = esp_timer_get_time();

        while (gpio_get_level(DHT_PIN) == 1)
        {
            if (esp_timer_get_time() - high_start > 100)
                return false;
        }

        int64_t pulse_length =
            esp_timer_get_time() - high_start;

        int byte_index = i / 8;
        int bit_index = 7 - (i % 8);

        if (pulse_length > 40)
        {
            data[byte_index] |= (1 << bit_index);
        }
    }

    // Verify checksum
    uint8_t checksum =
        data[0] +
        data[1] +
        data[2] +
        data[3];

    if (checksum != data[4])
        return false;

    // Humidity
    *humidity =
        ((data[0] << 8) | data[1]) / 10.0f;

    // Temperature
    int16_t raw_temperature =
        (data[2] << 8) | data[3];

    if (raw_temperature & 0x8000)
    {
        raw_temperature &= 0x7FFF;

        *temperature =
            -(raw_temperature / 10.0f);
    }
    else
    {
        *temperature =
            raw_temperature / 10.0f;
    }

    return true;
}

// ----------------------------------------------------
// SensorTask
// ----------------------------------------------------
void sensorTask(void *parameter)
{
    float temperature;
    float humidity;

    // ADC handle
    adc_oneshot_unit_handle_t adc1_handle;

    // ADC unit configuration
    adc_oneshot_unit_init_cfg_t init_config = {};

    init_config.unit_id = ADC_UNIT_1;

    adc_oneshot_new_unit(
        &init_config,
        &adc1_handle
    );

    // LDR ADC configuration
    adc_oneshot_chan_cfg_t adc_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };

    adc_oneshot_config_channel(
        adc1_handle,
        LDR_CHANNEL,
        &adc_config
    );

    // Used for periodic execution
    TickType_t lastWakeTime =
        xTaskGetTickCount();

    while (1)
    {
        SensorData sensorData;

        // --------------------------------------------
        // Read DHT22
        // --------------------------------------------
        if (dht22_read(&temperature, &humidity))
        {
            sensorData.temperature = temperature;
            sensorData.humidity = humidity;

            printf(
                "Temperature: %.2f C\n",
                temperature
            );

            printf(
                "Humidity: %.2f %%\n",
                humidity
            );
        }
        else
        {
            printf("DHT22 reading failed\n");

            sensorData.temperature = 0;
            sensorData.humidity = 0;
        }

        // --------------------------------------------
        // Read LDR
        // --------------------------------------------
        int raw_ldr = 0;

        esp_err_t result =
            adc_oneshot_read(
                adc1_handle,
                LDR_CHANNEL,
                &raw_ldr
            );

        if (result == ESP_OK)
        {
            float light_percent =
                (raw_ldr / 4095.0f) * 100.0f;

            sensorData.lightLevel =
                (int)light_percent;

            printf(
                "Light: %.2f %%\n",
                light_percent
            );
        }
        else
        {
            printf("LDR reading failed\n");

            sensorData.lightLevel = 0;
        }

        // Motion sensor will be added later
        sensorData.motionDetected = false;

        // --------------------------------------------
        // Send Sensor Data to Queue
        // --------------------------------------------
        if (xQueueSend(
                sensorQueue,
                &sensorData,
                pdMS_TO_TICKS(100)
            ) == pdPASS)
        {
            printf(
                "Sensor data sent to queue\n"
            );
        }
        else
        {
            printf(
                "Failed to send sensor data to queue\n"
            );
        }

        printf("SensorTask waiting 2 sec\n");

        // Keep a regular 2-second period
        vTaskDelayUntil(
            &lastWakeTime,
            pdMS_TO_TICKS(2000)
        );
    }
}

// ----------------------------------------------------
// DisplayTask
// ----------------------------------------------------
void displayTask(void *parameter)
{
    SensorData sensorData;

    char temperatureText[20];

    // Initialize OLED
    oled_init();

    // Clear OLED
    oled_clear();

    printf("DisplayTask started\n");

    while (1)
    {
        // Wait for sensor data from SensorTask
        if (xQueueReceive(
                sensorQueue,
                &sensorData,
                portMAX_DELAY
            ) == pdPASS)
        {
            // Clear previous display
            oled_clear();

            // Line 1
            oled_set_cursor(0, 0);

            oled_write_string(
                "ROOM MONITOR"
            );

            // Line 2
            oled_set_cursor(2, 0);

            oled_write_string(
                "Temperature"
            );

            // Convert temperature to text
            snprintf(
                temperatureText,
                sizeof(temperatureText),
                "%.1f C",
                sensorData.temperature
            );

            // Line 3
            oled_set_cursor(4, 0);

            oled_write_string(
                temperatureText
            );

            printf(
                "DisplayTask: Temperature %.1f C\n",
                sensorData.temperature
            );
        }
    }
}

// ----------------------------------------------------
// Main
// ----------------------------------------------------
extern "C" void app_main(void)
{
    printf("\n");

    printf(
        "BCA152 FreeRTOS Multisensor\n"
    );

    printf(
        "System starting...\n"
    );

    // Configure DHT22 pin
    gpio_set_direction(
        DHT_PIN,
        GPIO_MODE_INPUT
    );

    gpio_pullup_en(DHT_PIN);

    // --------------------------------------------
    // Create Sensor Queue
    // --------------------------------------------
    sensorQueue = xQueueCreate(
        5,
        sizeof(SensorData)
    );

    if (sensorQueue == NULL)
    {
        printf(
            "Failed to create sensor queue\n"
        );

        return;
    }

    printf(
        "Sensor queue created successfully\n"
    );

    // --------------------------------------------
    // Create SensorTask
    // --------------------------------------------
    xTaskCreate(
        sensorTask,
        "SensorTask",
        4096,
        NULL,
        2,
        NULL
    );

    // --------------------------------------------
    // Create DisplayTask
    // --------------------------------------------
    xTaskCreate(
        displayTask,
        "DisplayTask",
        4096,
        NULL,
        1,
        NULL
    );
}