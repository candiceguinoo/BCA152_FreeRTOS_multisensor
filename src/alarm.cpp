
#include "alarm.h"

// ====================================================
// ALARM DECISION LOGIC (pure - compiled for unit tests too)
// ====================================================

namespace
{
    constexpr float LOW_TEMPERATURE_LIMIT = 18.0f;
    constexpr float HIGH_TEMPERATURE_LIMIT = 30.0f;
}

AlarmState evaluateTemperature(float temperature)
{
    if (temperature < LOW_TEMPERATURE_LIMIT)
    {
        return AlarmState::LOW_TEMPERATURE;
    }

    if (temperature > HIGH_TEMPERATURE_LIMIT)
    {
        return AlarmState::HIGH_TEMPERATURE;
    }

    return AlarmState::NORMAL;
}

const char *alarmStateToString(AlarmState state)
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
// HARDWARE SIDE: buzzer driver + AlarmTask
// Excluded when PlatformIO builds the PC unit tests.
// ====================================================

#ifndef PIO_UNIT_TESTING

#include "driver/gpio.h"
#include "driver/ledc.h"

#include "rtos_objects.h"
#include "sensors.h"

// Buzzer (driven by LEDC PWM; owned exclusively by AlarmTask)
#define BUZZER_PIN              GPIO_NUM_26
#define BUZZER_LEDC_MODE        LEDC_LOW_SPEED_MODE
#define BUZZER_LEDC_TIMER       LEDC_TIMER_0
#define BUZZER_LEDC_CHANNEL     LEDC_CHANNEL_0
#define BUZZER_DUTY_ON          512
#define BUZZER_FREQ_HIGH_TEMP   2000
#define BUZZER_FREQ_LOW_TEMP    800
#define BUZZER_BEEP_MS          250

// ====================================================
// BUZZER DRIVER
// ====================================================

static void buzzer_init()
{
    ledc_timer_config_t timer = {};
    timer.speed_mode      = BUZZER_LEDC_MODE;
    timer.duty_resolution = LEDC_TIMER_10_BIT;
    timer.timer_num       = BUZZER_LEDC_TIMER;
    timer.freq_hz         = BUZZER_FREQ_HIGH_TEMP;
    timer.clk_cfg         = LEDC_AUTO_CLK;
    ledc_timer_config(&timer);

    ledc_channel_config_t channel = {};
    channel.gpio_num    = BUZZER_PIN;
    channel.speed_mode  = BUZZER_LEDC_MODE;
    channel.channel     = BUZZER_LEDC_CHANNEL;
    channel.intr_type   = LEDC_INTR_DISABLE;
    channel.timer_sel   = BUZZER_LEDC_TIMER;
    channel.duty        = 0;
    channel.hpoint      = 0;
    ledc_channel_config(&channel);
}

static void buzzer_on(uint32_t frequencyHz)
{
    ledc_set_freq(
        BUZZER_LEDC_MODE,
        BUZZER_LEDC_TIMER,
        frequencyHz
    );

    ledc_set_duty(
        BUZZER_LEDC_MODE,
        BUZZER_LEDC_CHANNEL,
        BUZZER_DUTY_ON
    );

    ledc_update_duty(
        BUZZER_LEDC_MODE,
        BUZZER_LEDC_CHANNEL
    );
}

static void buzzer_off()
{
    ledc_set_duty(
        BUZZER_LEDC_MODE,
        BUZZER_LEDC_CHANNEL,
        0
    );

    ledc_update_duty(
        BUZZER_LEDC_MODE,
        BUZZER_LEDC_CHANNEL
    );
}

static uint32_t buzzerFrequencyFor(AlarmState state)
{
    return (state == AlarmState::HIGH_TEMPERATURE)
        ? BUZZER_FREQ_HIGH_TEMP
        : BUZZER_FREQ_LOW_TEMP;
}

// ====================================================
// ALARM TASK
// ====================================================

void alarmTask(void *parameter)
{
    (void)parameter;

    SensorData sensorData = {};

    AlarmState lastState = AlarmState::NORMAL;
    AlarmState soundingState = AlarmState::NORMAL;

    bool beepOn = false;
    bool firstEvaluation = true;

    serialPrintf(
        "AlarmTask: Started on CPU %d\n",
        xPortGetCoreID()
    );

    while (1)
    {
        TickType_t waitTime =
            (soundingState == AlarmState::NORMAL)
                ? portMAX_DELAY
                : pdMS_TO_TICKS(BUZZER_BEEP_MS);

        if (
            xQueueReceive(
                alarmQueue,
                &sensorData,
                waitTime
            ) != pdPASS
        )
        {
            // Timeout: only happens while an alarm is sounding.
            if (
                (xEventGroupGetBits(systemEvents)
                 & EVENT_ACTIVE) == 0
            )
            {
                buzzer_off();
                beepOn = false;
                soundingState = AlarmState::NORMAL;
                firstEvaluation = true;

                serialPrintf(
                    "Buzzer: OFF - system inactive\n"
                );
            }
            else
            {
                beepOn = !beepOn;

                if (beepOn)
                {
                    buzzer_on(
                        buzzerFrequencyFor(soundingState)
                    );
                }
                else
                {
                    buzzer_off();
                }
            }

            continue;
        }

        // Alarm is active only when EVENT_ACTIVE is set.
        if (
            (xEventGroupGetBits(systemEvents)
             & EVENT_ACTIVE) == 0
        )
        {
            xEventGroupClearBits(
                systemEvents,
                EVENT_ALARM
            );

            if (soundingState != AlarmState::NORMAL)
            {
                buzzer_off();
                beepOn = false;
                soundingState = AlarmState::NORMAL;

                serialPrintf(
                    "Buzzer: OFF - system inactive\n"
                );
            }

            firstEvaluation = true;
            continue;
        }

        AlarmState state =
            evaluateTemperature(sensorData.temperature);

        // UPDATE EVENT_ALARM
        EventBits_t currentEvents =
            xEventGroupGetBits(systemEvents);

        bool alarmActive =
            state == AlarmState::LOW_TEMPERATURE ||
            state == AlarmState::HIGH_TEMPERATURE;

        if (alarmActive)
        {
            if ((currentEvents & EVENT_ALARM) == 0)
            {
                xEventGroupSetBits(
                    systemEvents,
                    EVENT_ALARM
                );

                serialPrintf("EVENT_ALARM SET\n");
            }
        }
        else
        {
            if (currentEvents & EVENT_ALARM)
            {
                xEventGroupClearBits(
                    systemEvents,
                    EVENT_ALARM
                );

                serialPrintf("EVENT_ALARM CLEARED\n");
            }
        }

        // ALARM STATE CHANGE -> BUZZER
        if (firstEvaluation || state != lastState)
        {
            serialPrintf(
                "Alarm State: %s (%.1f C)\n",
                alarmStateToString(state),
                sensorData.temperature
            );

            soundingState = state;

            if (alarmActive)
            {
                beepOn = true;

                buzzer_on(
                    buzzerFrequencyFor(state)
                );

                serialPrintf(
                    "Buzzer: ON (%u Hz)\n",
                    (unsigned)buzzerFrequencyFor(state)
                );
            }
            else
            {
                beepOn = false;
                buzzer_off();

                serialPrintf("Buzzer: OFF\n");
            }

            lastState = state;
            firstEvaluation = false;
        }
    }
}

void alarmInit()
{
    buzzer_init();
}

#endif // PIO_UNIT_TESTING
