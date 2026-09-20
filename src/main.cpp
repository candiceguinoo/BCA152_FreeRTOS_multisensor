#include <stdio.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

#include "esp_adc/adc_oneshot.h"

#define DHT_PIN GPIO_NUM_4

// LDR is connected to GPIO 34
// GPIO 34 = ADC1_CHANNEL_6
#define LDR_CHANNEL ADC_CHANNEL_6

// ----------------------------------------------------
// Sensor Data Structure
// ----------------------------------------------------
struct SensorData {
    float temperature;
    float humidity;
    int lightLevel;
    bool motionDetected;
};

// Sensor Queue
QueueHandle_t sensorQueue;

// ----------------------------------------------------
// DHT22 reading function
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
    TickType_t lastWakeTime = xTaskGetTickCount();

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

            printf("Temperature: %.2f C\n",
                   temperature);

            printf("Humidity: %.2f %%\n",
                   humidity);
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

            printf("Light: %.2f %%\n",
                   light_percent);
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
            printf("Sensor data sent to queue\n");
        }
        else
        {
            printf("Failed to send sensor data to queue\n");
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
// Main
// ----------------------------------------------------
extern "C" void app_main(void)
{
    printf("\n");
    printf("BCA152 FreeRTOS Multisensor\n");
    printf("System starting...\n");

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
        printf("Failed to create sensor queue\n");
        return;
    }

    printf("Sensor queue created successfully\n");

    // --------------------------------------------
    // Create SensorTask
    // --------------------------------------------
    xTaskCreate(
        sensorTask,
        "SensorTask",
        4096,
        NULL,
        1,
        NULL
    );
}