#include <stdio.h>

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"

#include "sensors.h"
#include "rtos_objects.h"
#include "system_state.h"

// ====================================================
// PIN AND TIMING CONSTANTS (this module only)
// ====================================================

#define DHT_PIN GPIO_NUM_4

#define LDR_ADC_CHANNEL ADC_CHANNEL_6   // GPIO34

#define DHT_TIMEOUT_US 150
#define DHT_BIT_TIMEOUT_US 120
#define DHT_ONE_THRESHOLD_US 50

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

    // Diagnostics are recorded inside the critical section and printed
    // AFTER it. serialPrintf() takes a mutex and writes to the UART,
    // which must never happen while interrupts are disabled.
    enum DhtProblem
    {
        DHT_PROBLEM_NONE,
        DHT_PROBLEM_INVALID_VALUES,
        DHT_PROBLEM_CHECKSUM,
        DHT_PROBLEM_TIMING
    };

    DhtProblem problem = DHT_PROBLEM_NONE;
    float badTemperature = 0.0f;
    float badHumidity = 0.0f;
    unsigned badCalcChecksum = 0;
    unsigned badRecvChecksum = 0;
    bool noResponse = false;

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
                        problem = DHT_PROBLEM_INVALID_VALUES;
                        badTemperature = newTemperature;
                        badHumidity = newHumidity;
                    }
                }
                else
                {
                    problem = DHT_PROBLEM_CHECKSUM;
                    badCalcChecksum = checksum;
                    badRecvChecksum = data[4];
                }
            }
            else
            {
                problem = DHT_PROBLEM_TIMING;
            }
        }
    }

    noResponse =
        !success &&
        !(responseLow && responseHigh);

    gpio_set_direction(
        DHT_PIN,
        GPIO_MODE_INPUT
    );

    gpio_pullup_en(DHT_PIN);

    portEXIT_CRITICAL(&dhtMux);

    // ---- critical section ended: safe to print now ----

    switch (problem)
    {
        case DHT_PROBLEM_INVALID_VALUES:
            serialPrintf(
                "DHT22 invalid values: "
                "T=%.2f C H=%.2f %%\n",
                badTemperature,
                badHumidity
            );
            break;

        case DHT_PROBLEM_CHECKSUM:
            serialPrintf(
                "DHT22 checksum failed "
                "(calc=%u received=%u)\n",
                badCalcChecksum,
                badRecvChecksum
            );
            break;

        case DHT_PROBLEM_TIMING:
            serialPrintf(
                "DHT22 timing/read failed\n"
            );
            break;

        case DHT_PROBLEM_NONE:
            break;
    }

    if (noResponse)
    {
        serialPrintf(
            "DHT22 sensor response timeout\n"
        );
    }

    return success;
}


// ====================================================
// INITIALISATION
// ====================================================

void sensorsInit()
{
    // DHT22 idle state: input with pull-up (line released).
    gpio_set_direction(
        DHT_PIN,
        GPIO_MODE_INPUT
    );

    gpio_pullup_en(
        DHT_PIN
    );
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
        serialPrintf(
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
        serialPrintf(
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

    bool haveValidDhtReading =
        false;

    serialPrintf(
        "SensorTask: Started on CPU %d\n",
        xPortGetCoreID()
    );

    while (1)
    {
        float newTemperature =
            0.0f;

        float newHumidity =
            0.0f;


        bool dhtOk =
            dht22_read(
                &newTemperature,
                &newHumidity
            );


        if (dhtOk)
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
                serialPrintf(
                    "DHT22 reading failed - "
                    "keeping previous valid reading\n"
                );
            }
            else
            {
                serialPrintf(
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
            // --------------------------------------------
            // LDR LIGHT LEVEL
            // Relative scale only: the raw ADC value is inverted
            // (the Wokwi LDR module reads LOW in bright light) and
            // mapped to 0-100 %. This is NOT calibrated lux.
            // --------------------------------------------

            const int LDR_RAW_MAX = 4095;

            int reversedRaw = LDR_RAW_MAX - raw_value;

            // Very low light = exactly 0%
            if (reversedRaw <= 40)
            {
                sensorData.lightLevel = 0;
            }
            // Maximum light = exactly 100%
            else if (reversedRaw >= 4000)
            {
                sensorData.lightLevel = 100;
            }
            // Normal range
            else
            {
                sensorData.lightLevel =
                    (int)(
                        (reversedRaw / 4095.0f)
                        * 100.0f
                    );
            }

            // Safety limits
            if (sensorData.lightLevel < 0)
                sensorData.lightLevel = 0;

            if (sensorData.lightLevel > 100)
                sensorData.lightLevel = 100;

            serialPrintf(
                "LDR Raw: %d | Reversed: %d | Light Level: %d %%\n",
                raw_value,
                reversedRaw,
                sensorData.lightLevel
            );
        }
        else
        {
            sensorData.lightLevel = 0;
        }

        sensorData.motionDetected =
            motionDetected;

        serialPrintf(
            "Temperature: %.2f C\n",
            sensorData.temperature
        );

        serialPrintf(
            "Humidity: %.2f %%\n",
            sensorData.humidity
        );

        serialPrintf(
            "Light Level: %d %%\n",
            sensorData.lightLevel
        );

        serialPrintf(
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

            serialPrintf(
                "Sensor data sent to queues\n"
            );
        }

        vTaskDelayUntil(
            &lastWakeTime,
            pdMS_TO_TICKS(2000)
        );
    }
}