
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "rtos_objects.h"
#include "sensors.h"
#include "display.h"
#include "input.h"
#include "motion.h"
#include "alarm.h"

// main.cpp only wires the system together:
//
//   hardware initialization
//        |
//   FreeRTOS object creation
//        |
//   task creation
//        |
//   scheduler-driven operation (app_main returns)

// Creates one pinned task and reports failure by name.
static bool startTask(
    TaskFunction_t function,
    const char *name,
    uint32_t stackSize,
    UBaseType_t priority,
    BaseType_t core
)
{
    BaseType_t result =
        xTaskCreatePinnedToCore(
            function,
            name,
            stackSize,
            NULL,
            priority,
            NULL,
            core
        );

    if (result != pdPASS)
    {
        serialPrintf(
            "ERROR: %s creation failed\n",
            name
        );

        return false;
    }

    return true;
}

extern "C" void app_main(void)
{
    serialPrintf("\n");

    serialPrintf(
        "====================================\n"
    );

    serialPrintf(
        "BCA152 FreeRTOS Multisensor\n"
    );

    serialPrintf(
        "System starting...\n"
    );

    serialPrintf(
        "====================================\n"
    );

    // --------------------------------------------
    // 1. HARDWARE INITIALIZATION
    // --------------------------------------------
    //
    // The OLED/I2C bus, ADC and encoder interrupt are
    // initialized by the task that owns them
    // (DisplayTask, SensorTask, InputTask).

    sensorsInit();   // DHT22 line idle
    motionInit();    // PIR input
    alarmInit();     // Buzzer (silent)

    // --------------------------------------------
    // 2. FREERTOS OBJECT CREATION
    // --------------------------------------------

    if (!rtosObjectsCreate())
    {
        return;
    }

    // --------------------------------------------
    // SYSTEM STARTS ACTIVE
    // --------------------------------------------

    xEventGroupSetBits(
        systemEvents,
        EVENT_ACTIVE
    );

    serialPrintf(
        "EVENT_ACTIVE SET - system startup\n"
    );

    // --------------------------------------------
    // 3. TASK CREATION
    // --------------------------------------------
    //
    // CPU0:
    // MotionTask  (priority 3)
    // SensorTask  (priority 2)
    //
    // CPU1:
    // InputTask   (priority 3)
    // AlarmTask   (priority 2)
    // DisplayTask (priority 1)

    if (!startTask(
        sensorTask, "SensorTask", 6144,
        PRIORITY_SENSOR, 0
    )) return;

    if (!startTask(
        motionTask, "MotionTask", 4096,
        PRIORITY_MOTION, 0
    )) return;

    if (!startTask(
        inputTask, "InputTask", 4096,
        PRIORITY_INPUT, 1
    )) return;

    if (!startTask(
        displayTask, "DisplayTask", 4096,
        PRIORITY_DISPLAY, 1
    )) return;

    if (!startTask(
        alarmTask, "AlarmTask", 4096,
        PRIORITY_ALARM, 1
    )) return;

    serialPrintf(
        "All tasks started successfully.\n"
    );

    // app_main returns; the FreeRTOS scheduler
    // keeps the five tasks running.
}
