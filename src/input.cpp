#include "input.h"

// ====================================================
// PAGE NAVIGATION (pure logic - compiled for unit tests too)
// ====================================================

DisplayMode nextDisplayMode(DisplayMode mode)
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

DisplayMode previousDisplayMode(DisplayMode mode)
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

const char *displayModeToString(DisplayMode mode)
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
// HARDWARE SIDE: encoder ISR + InputTask
// Excluded when PlatformIO builds the PC unit tests.
// ====================================================

#ifndef PIO_UNIT_TESTING

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_err.h"

#include "rtos_objects.h"

#define ENCODER_CLK GPIO_NUM_32
#define ENCODER_DT  GPIO_NUM_33
#define ENCODER_SW  GPIO_NUM_25

constexpr bool ENCODER_REVERSE = false;

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

    gpio_pullup_en(
        ENCODER_CLK
    );

    gpio_pullup_en(
        ENCODER_DT
    );

    gpio_pullup_en(
        ENCODER_SW
    );

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
        serialPrintf(
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
        gpio_get_level(
            ENCODER_SW
        );

    serialPrintf(
        "InputTask: Started on CPU %d\n",
        xPortGetCoreID()
    );

    while (1)
    {
        // --------------------------------------------
        // ROTATION
        // --------------------------------------------

        int8_t direction = 0;

        if (
            xQueueReceive(
                encoderQueue,
                &direction,
                ms_to_ticks(20)
            ) == pdPASS
        )
        {
            // Encoder is active only when EVENT_ACTIVE is set.
            EventBits_t events =
                xEventGroupGetBits(systemEvents);

            if (events & EVENT_ACTIVE)
            {
                if (direction > 0)
                {
                    currentMode =
                        nextDisplayMode(currentMode);

                    serialPrintf(
                        "InputTask: clockwise -> %s\n",
                        displayModeToString(currentMode)
                    );
                }
                else
                {
                    currentMode =
                        previousDisplayMode(currentMode);

                    serialPrintf(
                        "InputTask: counter-clockwise -> %s\n",
                        displayModeToString(currentMode)
                    );
                }

                xQueueOverwrite(
                    modeQueue,
                    &currentMode
                );
            }
        }

        // --------------------------------------------
        // ENCODER BUTTON
        // --------------------------------------------

        bool currentButton =
            gpio_get_level(
                ENCODER_SW
            );

        if (
            lastButton == 1 &&
            currentButton == 0 &&
            (xEventGroupGetBits(systemEvents) & EVENT_ACTIVE)
        )
        {
            currentMode =
                nextDisplayMode(currentMode);

            xQueueOverwrite(
                modeQueue,
                &currentMode
            );

            serialPrintf(
                "InputTask: button pressed -> %s\n",
                displayModeToString(currentMode)
            );

            vTaskDelay(
                ms_to_ticks(200)
            );
        }

        lastButton =
            currentButton;
    }
}

#endif // PIO_UNIT_TESTING
