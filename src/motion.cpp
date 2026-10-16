#include "driver/gpio.h"
#include "esp_timer.h"

#include "motion.h"
#include "rtos_objects.h"
#include "system_state.h"

// PIR motion sensor
#define PIR_PIN GPIO_NUM_27

void motionInit()
{
    gpio_set_direction(
        PIR_PIN,
        GPIO_MODE_INPUT
    );

    gpio_pulldown_en(
        PIR_PIN
    );
}

// ====================================================
// MOTION TASK
// ====================================================
//
// Reads the PIR, then asks evaluateSystemState() (pure logic in
// system_state.cpp) what the new ACTIVE/INACTIVE state should be.
// This task performs the side effects of a state change: event bits
// and diagnostics.

void motionTask(void *parameter)
{
    int64_t lastMotionTime =
        esp_timer_get_time();

    bool lastPirLevel =
        false;

    serialPrintf(
        "MotionTask: Started on CPU %d\n",
        xPortGetCoreID()
    );

    while (1)
    {
        bool pirLevel =
            gpio_get_level(PIR_PIN) != 0;

        int64_t now =
            esp_timer_get_time();

        // --------------------------------------------
        // PIR ACTIVITY
        // --------------------------------------------

        if (pirLevel)
        {
            lastMotionTime =
                now;

            motionDetected =
                true;

            if (!lastPirLevel)
            {
                serialPrintf(
                    "MotionTask: MOTION DETECTED\n"
                );

                xEventGroupSetBits(
                    systemEvents,
                    EVENT_MOTION
                );

                serialPrintf(
                    "EVENT_MOTION SET\n"
                );

                xEventGroupSetBits(
                    systemEvents,
                    EVENT_ACTIVE
                );

                serialPrintf(
                    "EVENT_ACTIVE SET\n"
                );
            }
        }

        lastPirLevel =
            pirLevel;

        // --------------------------------------------
        // STATE MACHINE
        // --------------------------------------------

        SystemState currentState =
            systemState;

        SystemState newState =
            evaluateSystemState(
                currentState,
                pirLevel,
                now - lastMotionTime
            );

        if (newState != currentState)
        {
            if (newState == SystemState::ACTIVE)
            {
                systemState =
                    SystemState::ACTIVE;

                serialPrintf(
                    "System entering ACTIVE mode\n"
                );
            }
            else
            {
                motionDetected =
                    false;

                systemState =
                    SystemState::INACTIVE;

                serialPrintf(
                    "MotionTask: NO MOTION - "
                    "15 SECOND TIMEOUT\n"
                );

                serialPrintf(
                    "System entering INACTIVE mode\n"
                );

                xEventGroupClearBits(
                    systemEvents,
                    EVENT_ACTIVE
                );

                serialPrintf(
                    "EVENT_ACTIVE CLEARED\n"
                );

                xEventGroupClearBits(
                    systemEvents,
                    EVENT_MOTION
                );

                serialPrintf(
                    "EVENT_MOTION CLEARED\n"
                );

                xEventGroupClearBits(
                    systemEvents,
                    EVENT_ALARM
                );

                serialPrintf(
                    "EVENT_ALARM CLEARED - system inactive\n"
                );
            }
        }

        vTaskDelay(
            ms_to_ticks(100)
        );
    }
}